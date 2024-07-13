#!/usr/bin/env python3


import os
import sys
import rospy
import asyncio
import rosnode
import logging
from utils import *
from RuntimeAssessmentConfig import RuntimeAssessmentConfig
from GlobalEvents import GlobalEvents
from datetime import datetime
from typing import Set
from AssessmentObjects import AssessmentObject


class RuntimeAssessment:
    """
    Class to assess the runtime behavior of a ROS application.
    """

    def __init__(self, config: RuntimeAssessmentConfig):
        # Initialize node
        try:
            self.node = rospy.init_node('runtime_assessment')
        except rospy.ROSException as e:
            sys.exit(f"ROS Exception: {e}")

        # Declare setup variables
        self.target_node = config.target_node
        self.topics = config.topics
        self.rate = rospy.Rate(config.rate)
        self.logger_path = config.logger_path
        self.topic_pairs = {}
        self.start_time = float()

        self.metrics = {'execution_time': float(), 'number_of_messages': float(), 'frequency': float()}

        self.metrics_assessment_pool = []

        # Get specifications
        self.specifications = config.specifications

        # Initialize internal state variables
        self.previous_nodes: Set[str] = set()
        self.is_paused = False
        self.is_running = False
        self.assessment_over = False

        # Initialize queue for events that is accessed concurrently by the assessment objects
        self.global_event_queue = []

        # Create lock for thread-safe access to the global_event_queue
        self.lock = asyncio.Lock()

        # Logger setup
        self.logger = logging.getLogger(f"RuntimeAssessment.{self.target_node}")

        # Get current date for file name generation
        current_date = datetime.now().strftime("%Y_%m_%d")

        # Create target path if it does not exist
        log_dir = os.path.join(self.logger_path, f"{self.target_node.split('/')[1]}_{current_date}")
        os.makedirs(log_dir, exist_ok=True)

        # Create shared file handler
        log_file_path = os.path.join(log_dir, f"{self.target_node.split('/')[1]}_assessment.log")
        self.file_handler = logging.FileHandler(log_file_path)
        file_formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        self.file_handler.setFormatter(file_formatter)
        self.logger.addHandler(self.file_handler)

        # INFO and Above Handler
        info_handler = logging.FileHandler(os.path.join(log_dir, f"{self.target_node.split('/')[1]}_assessment_info.log"))
        info_formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        info_handler.setFormatter(info_formatter)
        info_handler.setLevel(logging.INFO)
        self.logger.addHandler(info_handler)

        # ERROR and Above Handler
        error_handler = logging.FileHandler(os.path.join(log_dir, f"{self.target_node.split('/')[1]}_assessment_error.log"))
        error_formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        error_handler.setFormatter(error_formatter)
        error_handler.setLevel(logging.ERROR)
        self.logger.addHandler(error_handler)

        self.logger.info(" ------------ RUNTIME ASSESSMENT ------------ ")

        # import required message types for the assessments
        for topic in self.topics.items():
            self.topic_pairs[topic[0]] = import_message_type(topic)

        # create an assessment pool and initialize assessment objects
        self.assessment_pool = []
        self.assessment_object_allocator()

    async def publish_global_event(self, event: GlobalEvents) -> None:
        """
        Publish a global event in a thread-safe way.
        Enforce a size limit to the event queue.
        :param event: GlobalEvents
        """
        async with self.lock:
            self.global_event_queue.append((datetime.now(), event))
            print(f"Published Global Event: {event}.\n")

            # keep the queue size at maximum 10 events
            if len(self.global_event_queue) > 10:
                self.global_event_queue.pop(0)

    def run(self) -> None:
        """
        Run the assessment asynchronously.
        :return: None
        """
        asyncio.run(self.async_run())

    async def async_run(self) -> None:
        """
        Main loop that monitors the state of the target node and reacts to it.
        Acts as a state machine for the global system.
        # TODO: draw state-machine diagram
        :return: None
        """
        #
        assessment_tasks = [asyncio.create_task(obj.run()) for obj in self.assessment_pool]

        while not rospy.is_shutdown() and not self.assessment_over:
            target_running: bool = self.target_node in rosnode.get_node_names()

            # running state
            if self.is_running:
                # target stopped running event leads to ending the assessment
                if not target_running:
                    # push global event indicating new state to the assessment objects and wait for their conclusion
                    await self.publish_global_event(GlobalEvents.NODE_REMOVED)
                    await asyncio.gather(*assessment_tasks)
                    self.end_assessment()
                else:
                    await asyncio.sleep(self.rate.sleep_dur.to_sec())

            # idle state
            else:
                # target stopped running event leads to ending the assessment
                if target_running:
                    # push global event indicating new state to the assessment objects
                    await self.publish_global_event(GlobalEvents.NODE_ADDED)
                    self.start_assessment()
                # sleep at the specified rate until the target node is started
                else:
                    await asyncio.sleep(self.rate.sleep_dur.to_sec())

    def start_assessment(self) -> None:
        """
        Start the assessment.
        :return: None
        """
        # record start time of the assessment
        self.start_time = rospy.get_time()

        # update internal state
        self.is_paused = False
        self.is_running = True
        self.logger.info("Target node started. Initializing assessment...")

    def end_assessment(self) -> None:
        """
        End the assessment.
        :return: None
        """
        # compute global metrics
        self.metrics['execution_time'] = self.get_time_elapsed()
        self.metrics['number_of_messages'] = self.aggregate_message_counts()
        self.metrics['frequency'] = self.metrics['number_of_messages'] / self.metrics['execution_time']

        # log global metrics
        self.logger.info("Target node removed. Finishing assessment...")
        self.logger.info(f"Execution time: {self.metrics['execution_time']} seconds.")
        self.logger.info(f"Total number of messages: {self.metrics['number_of_messages']}")
        self.logger.info(f"Frequency: {self.metrics['frequency']}")

        # assess global metrics requirements
        self.metric_assessment()

        # update internal state
        self.is_running = False
        self.is_paused = False
        self.logger.info("----------------- END OF ASSESSMENT -----------------\n")
        self.assessment_over = True

    def get_time_elapsed(self) -> float:
        """
        Compute the time elapsed since the assessment started.
        :return: float
        """
        return rospy.get_time() - self.start_time

    def assessment_object_allocator(self) -> None:
        """
        Allocate assessment objects based on the requirements
        :return: None
        """

        # Topic assessments
        for topic, requirements in self.specifications["topic"].items():
            # Map the corresponding message class
            message_class = self.topic_pairs[topic]

            # set default values to missing parameters
            for req in requirements:
                # set default values for missing keys
                req.setdefault('mode', "exists")
                req.setdefault('temporal_consistency', False)
                req.setdefault('timein', None)
                req.setdefault('timeout', None)
                req.setdefault('tolerance', 0.05)
                req.setdefault('comparator', "=")

            # initialize the assessment object for the current topic
            assessment_object = AssessmentObject(runtime_assessment=self, topic_name=topic, message_class=message_class,
                                                 requirements=requirements)

            # add the initialized object to the assessment pool
            self.assessment_pool.append(assessment_object)
            self.logger.info(
                f"Assessment object created for topic '{topic}' with message class {message_class} and {len(requirements)} requirements.")

        # Global metric assessments
        for metric, requirements in self.specifications["metric"].items():
            # set default values to missing parameters
            for req in requirements:
                # set default values for missing keys
                req.setdefault('mode', "total")
                req.setdefault('temporal_consistency', False)
                req.setdefault('timein', None)
                req.setdefault('timeout', None)
                req.setdefault('tolerance', 0.05)
                req.setdefault('comparator', "=")

            # append restructured dictionary of each metric assessment to assessment pool
            self.metrics_assessment_pool.append({'metric': metric, 'requirements': requirements})

    def metric_assessment(self):
        """
        Assess the global metrics.
        :return: None
        """
        # iterate all metric assessments scheduled
        for metric in self.metrics_assessment_pool:
            # validate desired metric assessment
            if metric['metric'] in list(self.metrics.keys()):
                self.logger.info(
                    f"Assessing metric '{metric['metric']}' with {len(metric['requirements'])} requirements.")

            else:
                self.logger.error(f"Metric '{metric['metric']}' not found.")
                continue

            # assess all requirements for current metric
            for i, req in enumerate(metric['requirements']):
                target = req['target']
                tolerance = req['tolerance']
                comparator = req['comparator']

                value = self.metrics[metric['metric']]

                # handle assessment value of type list
                if isinstance(target, list):
                    # default range values to None
                    min_val = None
                    max_val = None

                    # iterate assessment value parameters
                    for limit in target:
                        for k, v in limit.items():
                            # validate param
                            if not is_numeric(v):
                                self.logger.error(
                                    f"Requirement {i + 1} of {len(metric['requirements'])} of metric '{metric['metric']}' FAILED - Invalid target value.")
                                return

                            if k == "min":
                                min_val = v
                            elif k == "max":
                                max_val = v
                            else:
                                self.logger.error(
                                    f"Requirement {i + 1} of {len(metric['requirements'])} of metric '{metric['metric']}' FAILED - Invalid target value.")
                                return

                    # perform the check and log the results
                    try:
                        if check_value_params(value, (min_val, max_val), comparator, tolerance):
                            self.logger.info(
                                f"Requirement {i + 1} of {len(metric['requirements'])} of metric '{metric['metric']}' PASSED.")
                            continue
                        else:
                            self.logger.info(
                                f"Requirement {i + 1} of {len(metric['requirements'])} of metric '{metric['metric']}' FAILED.")
                            continue

                    except Exception as e:
                        self.logger.error(
                            f"Requirement {i + 1} of {len(metric['requirements'])} of metric '{metric['metric']}' FAILED - {e}")
                        return

                # handle assessment value of type float or int
                elif is_numeric(target):
                    try:
                        # perform the check and log the results
                        if check_value_params(value, target, tolerance, comparator):
                            self.logger.info(
                                f"Requirement {i + 1} of {len(metric['requirements'])} of metric '{metric['metric']}' PASSED.")
                            continue
                        else:
                            self.logger.info(
                                f"Requirement {i + 1} of {len(metric['requirements'])} of metric '{metric['metric']}' FAILED.")
                            continue

                    except Exception as e:
                        self.logger.error(
                            f"Requirement {i + 1} of {len(metric['requirements'])} of metric '{metric['metric']}' FAILED - {e}")
                        return
                else:
                    self.logger.error(
                        f"Requirement {i + 1} of {len(metric['requirements'])} of metric '{metric['metric']}' FAILED - Invalid target value.")
                    return

    def aggregate_message_counts(self) -> int:
        """
        Aggregate the total number of messages from all assessment objects.
        :return: int
        """
        return sum(obj.metrics['number_of_messages'] for obj in self.assessment_pool)