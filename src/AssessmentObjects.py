# Description: Contains the classes for the assessment objects.
from typing import Any, List, Tuple, Union
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import rospy
from GlobalEvents import GlobalEvents
from utils import ordered_points, unordered_points, get_average_value, frequency_of_events, has_attribute
from RuntimeAssessment import RuntimeAssessment


class AssessmentObject:
    """
    Class to create an assessment object.
    """
    def __init__(self, runtime_assessment: RuntimeAssessment, topic_name: str, message_class: Any, requirements: List) -> None:
        # runtime assessment hooks and variables
        self.node = runtime_assessment.node
        self.rate = runtime_assessment.rate
        self.logger = runtime_assessment.logger
        self.runtime_assessment = runtime_assessment

        # assessment object variables
        self.latest_global_event = Tuple()
        self.topic_name = topic_name
        self.message_class = message_class
        self.specifications = requirements
        self.latest_topic_event = self.message_class()
        self.topic_event_record = []
        

    def global_event_callback(self, event: Tuple) -> None:
        """
        Callback for the global events subscriber.
        :param data: GlobalEvents
        :return: None
        """
        # map the event to the data
        data = event[1]

        if data == GlobalEvents.NODE_ADDED:
            self.start_assessment()

        elif data == GlobalEvents.ASSESSMENT_PAUSED:
            self.remove_subscribers()

        elif data == GlobalEvents.ASSESSMENT_RESUMED:
            self.create_subscribers()

        elif data == GlobalEvents.NODE_REMOVED:
            self.end_assessment()

        else:
            self.logger.error(f"Invalid global event received: {data}")


    def create_subscribers(self) -> None:
        """
        Create the subscribers.
        :return: None
        """
        try:
            self.sub = rospy.Subscriber(self.topic_name, self.message_class, self.handle_sub, queue_size=10)

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
        if isinstance(data, self.message_class):
            self.save_record(self.topic_event_record, data)
            self.latest_topic_event = data
        
        else:
            self.logger.error(f"Invalid message type received: {type(data)}")


    def start_assessment(self) -> None:
        """
        Start the assessment.
        :return: None
        """
        self.create_subscribers()


    def end_assessment(self) -> None:
        """
        End the assessment.
        :return: None
        """
        self.remove_subscribers()
        self.check_requirements()

    
    def save_record(self, target: List, data: Any) -> None:
        """
        Save the data to the target list with a timestamp.
        :param target: List
        :param data: Any
        :return: None
        """
        try:
            target.append((self.get_time_elapsed(), data))

        except Exception as e:
            self.logger.error(e)
            
    
    def exists_on_record(self, target: List, record: List[Tuple], ordered: bool = False, tolerance: float = 0.05) -> bool:
        """
        Check if certain points occur in a target record.
        :param positions: List[Tuple]
        :param tolerance: float
        :return: bool
        """
        
        if ordered:
            try:
                return ordered_points(target, record, tolerance)
            except Exception as e:
                self.logger.error(e)
                return False
        else:
            try:
                return unordered_points(target, record, tolerance)
            except Exception as e:
                self.logger.error(e)
                return False
            

    def check_value_params(self, value: float, target: Union[float, Tuple], tolerance: float = 0.05, comp: str = "=") -> bool:
        """
        Check if the average velocity of the turtle is within the tolerance of the target.
        :param value: float
        :param target: float || Tuple
        :param tolerance: float
        :return: bool
        """
        
        if isinstance(target, tuple):
            min_target, max_target = target
            if min_target < value < max_target:
                return True
            
        elif isinstance(target, float):
            comparison_dict = {
                "=": lambda: target*(1 - tolerance) < value < target*(1 + tolerance),
                ">": lambda: value > target,
                "<": lambda: value < target,
                ">=": lambda: value >= target,
                "<=": lambda: value <= target,
                "!=": lambda: value != target,
            }

            if comp in comparison_dict:
                return comparison_dict[comp]()

            else:
                self.logger.error("Invalid comparison operator.")
        
        else:
            self.logger.error("Invalid target.")
            
        return False
    

    def check_requirements(self) -> bool:

        """
        Check if the requirements are met.
        :param requirements: List[Tuple]
        :return: bool
        """

        for req in self.requirements:
            mode = req["mode"]
            target = req["target"]
            tolerance = req["tolerance"]
            comparator = req["comparator"]
            temporal_consistency = req["temporal_consistency"]
            timein = req["timein"]
            timeout = req["timeout"]

            if mode == "exists":
                try:
                    if not self.exists_on_record(target, self.topic_event_record, ordered=temporal_consistency, tolerance=tolerance):
                        self.logger.error(f"Requirement {req} FAILED.")
                        break

                    else:
                        self.logger.info(f"Requirement {req} PASSED.")

                except ValueError as e:
                    self.logger.error(f"Requirement {req} FAILED - {e}")
                    break
                
                except Exception as e:
                    self.logger.error(f"Requirement {req} FAILED - {e}")
                    break


            elif mode == "absent":
                try:
                    if self.exists_on_record(target, self.topic_event_record, ordered=temporal_consistency, tolerance=tolerance):
                        self.logger.error(f"Requirement {req} FAILED.")
                        break

                    else:
                        self.logger.info(f"Requirement {req} PASSED.")

                except ValueError as e:
                    self.logger.error(f"Requirement {req} FAILED - {e}")
                    break
                
                except Exception as e:
                    self.logger.error(f"Requirement {req} FAILED - {e}")
                    break

            elif mode == "max":
                pass

            elif mode == "min":
                pass

            elif mode == "total":
                pass

            elif mode == "average":
                pass

            elif mode == "metric":
                pass

            else:
                self.logger.error(f"Requirement {req} FAILED - Invalid mode.")
        
        return True
    

    def run(self) -> None:
        """
        Run the assessment.
        :return: None
        """

        # check for new events
        if self.latest_global_event != self.runtime_assessment.global_event_queue[-1]:
            self.latest_global_event = self.runtime_assessment.global_event_queue[-1]
            self.global_event_callback(self.latest_global_event)

        while not rospy.is_shutdown():
            self.rate.sleep()