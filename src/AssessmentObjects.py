# Description: Contains the classes for the assessment objects.
import asyncio
import rospy
from GlobalEvents import GlobalEvents
from utils import *
import logging


def exists_on_record(target: List, record: List[Tuple], ordered: bool = False, tolerance: float = 0.05,
                     timein=None, timeout=None) -> bool:
    """
    Check if certain points occur in a target record.
    :param target: list
    :param record: list
    :param ordered: bool
    :param tolerance: float
    :param timein: float
    :param timeout: float
    :return: bool
    """
    if ordered:
        try:
            return ordered_points(target, record, tolerance, timein, timeout)
        except Exception as e:
            raise e
    else:
        try:
            return unordered_points(target, record, tolerance, timein, timeout)
        except Exception as e:
            raise e


class AssessmentObject:
    """
    Class to create an assessment object.
    """
    def __init__(self, runtime_assessment, topic_name: str, message_class: Any, requirements: List) -> None:
        # runtime assessment hooks and variables
        self.runtime_assessment = runtime_assessment
        self.node = runtime_assessment.node
        self.rate = runtime_assessment.rate

        # Setup logger
        self.logger = logging.getLogger(f"AssessmentObject.{topic_name}")
        self.logger.addHandler(runtime_assessment.file_handler)
        self.logger.setLevel(logging.INFO)

        # Assessment variables
        self.start_time = 0
        self.metrics = {'number_of_messages': float(), 'frequency': float()}
        self.topic_name = topic_name
        self.message_class = message_class
        self.requirements = requirements
        self.sub = None
        self.topic_event_record = []

        # Initialize internal state variables
        self.over = False
        self.latest_global_event = tuple()
        self.latest_topic_event = self.message_class()

        self.logger.info(f"Assessment object created for topic {self.topic_name}.")

    def global_event_callback(self, event: Tuple) -> None:
        """
        Callback for the global events' subscriber.
        :param event: GlobalEvents
        :return: None
        """
        # map the event to the data
        data = event[1]
        self.logger.info(f"Global event received: {data.name}")

        # handle respective event
        if data == GlobalEvents.NODE_ADDED:
            self.logger.info("Handling NODE_ADDED event.")
            self.start_assessment()

        elif data == GlobalEvents.ASSESSMENT_PAUSED:
            self.logger.info("Handling ASSESSMENT_PAUSED event.")
            self.remove_subscribers()

        elif data == GlobalEvents.ASSESSMENT_RESUMED:
            self.logger.info("Handling ASSESSMENT_RESUMED event.")
            self.create_subscribers()

        elif data == GlobalEvents.NODE_REMOVED:
            # extract internal metrics
            self.metrics['number_of_messages'] = len(self.topic_event_record)
            self.metrics['frequency'] = frequency_of_events(self.topic_event_record)

            # log the metrics
            self.logger.info("Handling NODE_REMOVED event.")
            self.logger.info(f"Number of events recorded: {self.metrics['number_of_messages']}")
            self.logger.info(f"Frequency of events: {self.metrics['frequency']} Hz")

            # trigger end assessment procedure
            self.end_assessment()

        else:
            self.logger.error(f"Invalid global event received: {data}")
            self.over = True

    def create_subscribers(self) -> None:
        """
        Create the subscribers.
        :return: None
        """
        try:
            self.sub = rospy.Subscriber(self.topic_name, self.message_class, self.handle_sub, queue_size=10)
            self.logger.info(f"Subscriber created for topic {self.topic_name}.")

        except Exception as e:
            self.logger.error(e)

    def remove_subscribers(self) -> None:
        """
        Remove the subscribers.
        :return: None
        """
        try:
            self.sub.unregister()

        except Exception as e:
            self.logger.error(e)

    def handle_sub(self, data) -> None:
        """
        Handle the subscribers.
        :return: None
        """
        # Validate if the received message is of the expected data type
        if isinstance(data, self.message_class):
            # Update record
            self.save_record(self.topic_event_record, data)
            self.latest_topic_event = data
            if len(self.topic_event_record) == 1:
                self.logger.info(f"Receiving data.")

        else:
            self.logger.error(f"Invalid message type received: {type(data)}")

    def start_assessment(self) -> None:
        """
        Start the assessment.
        :return: None
        """
        self.start_time = rospy.get_time()
        self.create_subscribers()

    def end_assessment(self) -> None:
        """
        End the assessment.
        :return: None
        """
        self.logger.info("Checking requirements.")
        self.check_requirements()
        self.logger.info("Unregistering subscriber.")
        self.remove_subscribers()

        # Update internal state to indicate end of the assessment
        self.over = True
        self.logger.info("Assessment ended.\n")

    def save_record(self, target: List, data: Any) -> None:
        """
        Save the data to the target list with a timestamp.
        :param target: List
        :param data: Any
        :return: None
        """
        try:
            # store a timestamped event of the data received
            target.append((self.get_time_elapsed(), data))

        except Exception as e:
            self.logger.error(e)

    def get_time_elapsed(self) -> float:
        """
        Compute the time elapsed since the assessment started.
        :return: float
        """
        return rospy.get_time() - self.start_time

    def check_requirements(self) -> bool:
        """
        Check if the requirements are met.
        :param requirements: List[Tuple]
        :return: bool
        """

        for i, req in enumerate(self.requirements):
            mode = req["mode"]
            target = req["target"]
            tolerance = req["tolerance"]
            comparator = req["comparator"]
            temporal_consistency = req["temporal_consistency"]
            timein = req["timein"]
            timeout = req["timeout"]

            if mode == "exists":
                record_filtered = filter_by_time(self.topic_event_record, timein, timeout)
                try:
                    if not exists_on_record(target, record_filtered, ordered=temporal_consistency,
                                            tolerance=tolerance, timein=timein, timeout=timeout):
                        self.logger.info(f"Requirement {i + 1} of {len(self.requirements)} FAILED. "
                                         f"Target: {target} not found on the recorded events.")
                        continue
                    else:
                        self.logger.info(f"Requirement {i + 1} of {len(self.requirements)} PASSED.")
                        continue
                except ValueError as e:
                    self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - {e}")
                    return
                except Exception as e:
                    self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - {e}")
                    return

            elif mode == "absent":
                record_filtered = filter_by_time(self.topic_event_record, timein, timeout)
                try:
                    if exists_on_record(target, record_filtered, ordered=temporal_consistency,
                                        tolerance=tolerance):
                        self.logger.info(f"Requirement {i + 1} of {len(self.requirements)} FAILED. "
                                         f"Target: {target} was found on the recorded events")
                        continue
                    else:
                        self.logger.info(f"Requirement {i + 1} of {len(self.requirements)} PASSED.")
                        continue
                except ValueError as e:
                    self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - {e}")
                    return
                except Exception as e:
                    self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - {e}")
                    return

            elif mode == "max" or mode == "min" or mode == "average" or mode == "metric":
                record_filtered = filter_by_time(self.topic_event_record, timein, timeout)
                attr, tgt_val = list(target[0].items())[0]

                if mode == "max":
                    value = get_max(attr, record_filtered)
                elif mode == "min":
                    value = get_min(attr, record_filtered)
                elif mode == "average":
                    value = get_average_value(attr, record_filtered)
                elif mode == "metric":
                    if attr in list(self.metrics.keys()):
                        value = self.metrics[attr]
                    else:
                        self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - Invalid metric. "
                                          f"Metric: {attr}")
                        return

                if isinstance(tgt_val, list):
                    min_val = None
                    max_val = None
                    for limit in tgt_val:
                        for k, v in limit.items():
                            if not is_numeric(v):
                                self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - Invalid target value. "
                                                  f"Target: {tgt_val}")
                                return
                            if k == "min":
                                min_val = v
                            elif k == "max":
                                max_val = v
                            else:
                                self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - Invalid target value. "
                                                  f"Target: {tgt_val}")
                                return
                    try:
                        if check_value_params(value, (min_val, max_val), comparator, tolerance):
                            self.logger.info(f"Requirement {i + 1} of {len(self.requirements)} PASSED.")
                            continue
                        else:
                            self.logger.info(f"Requirement {i + 1} of {len(self.requirements)} FAILED. "
                                             f"Value: {value}, Target: {tgt_val}, Comparator: {comparator}")
                            continue
                    except Exception as e:
                        self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - {e}")
                        return

                elif is_numeric(tgt_val):
                    try:
                        if check_value_params(value, tgt_val, tolerance, comparator):
                            self.logger.info(f"Requirement {i + 1} of {len(self.requirements)} PASSED.")
                            continue
                        else:
                            self.logger.info(f"Requirement {i + 1} of {len(self.requirements)} FAILED. "
                                             f"Value: {value}, Target: {tgt_val}, Comparator: {comparator}")
                            continue
                    except Exception as e:
                        self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - {e}")
                        return
                else:
                    self.logger.error(f"Requirement {i + 1} of {len(self.requirements)} FAILED - Invalid target value. "
                                      f"Target: {tgt_val}")
                    return

        return

    async def run(self) -> None:
        """
        Run the assessment.
        :return: None
        """
        while not rospy.is_shutdown() and not self.over:

            # check for new events
            async with self.runtime_assessment.lock:
                if len(self.runtime_assessment.global_event_queue) != 0:
                    if self.latest_global_event != self.runtime_assessment.global_event_queue[-1]:
                        self.latest_global_event = self.runtime_assessment.global_event_queue[-1]
                        self.global_event_callback(self.latest_global_event)

            await asyncio.sleep(self.rate.sleep_dur.to_sec())