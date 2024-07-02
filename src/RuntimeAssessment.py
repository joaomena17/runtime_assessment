#!/usr/bin/env python3


import os
import sys
import rospy
import rosnode
import logging
import rostopic
import importlib
from utils import *
from RuntimeAssessmentConfig import RuntimeAssessmentConfig
from GlobalEvents import GlobalEvents
from datetime import datetime
from typing import List, Tuple, Any, Set, Union
    

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
        self.rate = config.rate
        self.logger_path = config.logger_path
        self.topic_pairs = {}
        self.start_time = float()
        self.execution_time = float()
        self.number_of_messages = 0 # TODO: implement
        self.frequency = 0 # TODO: implement


        # Get specifications
        self.specifications = config.specifications

        # Initialize internal variables
        self.previous_nodes: Set[str] = set()
        self.is_paused = False
        self.is_running = False
        self.global_event_queue = []

        # Logger setup
        self.logger = logging.getLogger(f"RuntimeAssessment.{self.target_node}")

        current_date = datetime.now().strftime("%Y_%m_%d")
        log_dir = os.path.join(self.logger_path, f"{self.target_node.split('/')[1]}_{current_date}")
        os.makedirs(log_dir, exist_ok=True) 

        # All Levels Handler
        all_levels_handler = logging.FileHandler(os.path.join(log_dir, f"{self.target_node.split('/')[1]}_assessment.log"))
        all_levels_formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        all_levels_handler.setFormatter(all_levels_formatter)
        self.logger.addHandler(all_levels_handler)

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

        # import message types
        for topic in self.topics.items():
            self.topic_pairs[topic[0]] = import_message_type(topic)

        # TODO: initialize assessment objects
        self.assessment_pool = []
        self.assessment_object_allocator()
    

    def publish_global_event(self, event: GlobalEvents) -> None:
        """
        Publish a global event.
        :param event: GlobalEvents
        """
        self.global_event_queue.append((datetime.now(), event))

        # keep the queue size at maximum 10 events
        if len(self.global_event_queue) > 10:
            self.global_event_queue.pop(0)


    def run(self) -> None:
        """
        Run the assessment.
        :return: None
        """
        while not rospy.is_shutdown():
            target_running: bool = self.target_node in rosnode.get_node_names()

            if self.is_running:
                if self.is_paused:
                    if not target_running:
                        self.logger.info("Target node removed. Finishing assessment...")
                        self.publish_global_event(GlobalEvents.NODE_REMOVED)
                        self.end_assessment()
                    else:
                        self.logger.info("Assessment paused.")
                        self.publish_global_event(GlobalEvents.ASSESSMENT_PAUSED)
                        rospy.sleep(1.0)

                else:
                    if not target_running:
                        self.logger.info("Target node removed. Finishing assessment...")
                        self.publish_global_event(GlobalEvents.NODE_REMOVED)
                        self.end_assessment()
                    else:
                        self.rate.sleep()   
                
            else:
                if target_running:
                    self.logger.info("Target node started. Initializing assessment...")
                    self.publish_global_event(GlobalEvents.NODE_ADDED)
                    self.start_assessment()
                else:
                    self.rate.sleep()

    
    def pause(self):
        """
        Pause the assessment.
        :return: None
        """
        if self.is_running:
            self.is_paused = True
            self.logger.info("Assessment paused.")


    def resume(self):
        """
        Resume the assessment.
        :return: None
        """
        if self.is_paused:
            self.is_paused = False
            self.is_running = True
            self.publish_global_event(GlobalEvents.ASSESSMENT_RESUMED)
            self.logger.info("Assessment resumed.")


    def start_assessment(self) -> None:
        """
        Start the assessment.
        :return: None
        """
        self.start_time = rospy.get_time()
        self.is_paused = False
        self.is_running = True
        self.logger.info("Assessment started.")


    def end_assessment(self) -> None:
        """
        End the assessment.
        :return: None
        """
        rospy.signal_shutdown("Assessment stopped.")
        self.is_running = False
        self.is_paused = False
        self.total_execution_time = self.get_time_elapsed()
        self.logger.info("Assessment ended.")
        self.logger.info(f"Execution time: {self.execution_time} seconds.")
        self.logger.info("----------------- END OF ASSESSMENT -----------------\n")


    def get_time_elapsed(self) -> float:
        """
        Get the time elapsed since the assessment started.
        :return: float
        """
        return rospy.get_time() - self.start_time
    

    def assessment_object_allocator(self):
        """
        Allocate assessment objects based on the requirements
        :param requirements: A dictionary of requirements
        :return: A dictionary of assessment objects
        """

        # Topic assessments
        for topic, requirements in self.specifications["topic"].items():

            # Parameters to create the assessment object
            message_class = self.topic_pairs[topic]            

            for req in requirements:   
                # set default values for missing keys
                req.setdefault('mode', "exists")
                req.setdefault('temporal_consistency', False)
                req.setdefault('timein', None)
                req.setdefault('timeout', None)
                req.setdefault('tolerance', 0.05)
                req.setdefault('comparator', "=")
                req.setdefault('min', None)
                req.setdefault('max', None)

            # TODO: create an assessment object for this topic

        # Global metric assessments
        for metric, requirements in self.specifications["metric"].items():
                # set default values for missing keys
                req.setdefault('mode', "total")
                req.setdefault('temporal_consistency', False)
                req.setdefault('timein', None)
                req.setdefault('timeout', None)
                req.setdefault('tolerance', 0.05)
                req.setdefault('comparator', "=")
                req.setdefault('min', None)
                req.setdefault('max', None)

            # TODO: create an assessment object for this metric
        