# Description: Contains the classes for the assessment objects.
from typing import Any, List, Tuple, Union
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import rospy
from GlobalEvents import GlobalEvents
from utils import ordered_points, unordered_points, get_average_value, frequency_of_events
from RuntimeAssessment import RuntimeAssessment


class AssessmentObject:
    """
    Class to create an assessment object.
    """
    def __init__(self, runtime_assessment: RuntimeAssessment):
        self.node = runtime_assessment.node
        self.rate = runtime_assessment.rate
        self.logger = runtime_assessment.logger
        self.subscribers = []
        self.specifications = []
        

    def global_event_callback(self, data: GlobalEvents) -> None:
        """
        Callback for the global events subscriber.
        :param data: GlobalEvents
        :return: None
        """
        if data == GlobalEvents.NODE_ADDED:
            self.start_assessment()

        elif data == GlobalEvents.ASSESSMENT_PAUSED:
            self.remove_subscribers()

        elif data == GlobalEvents.ASSESSMENT_RESUMED:
            self.create_subscribers()

        elif data == GlobalEvents.NODE_REMOVED:
            self.end_assessment()


    def create_subscribers(self) -> None:
        """
        Create the subscribers.
        :return: None
        """
        pass

    
    def remove_subscribers(self):
        """
        Remove the subscribers.
        :return: None
        """
        for sub in self.subscribers:
            sub.unregister()


    def handle_sub():
        """
        Handle the subscribers.
        :return: None
        """
        pass


    def start_assessment(self) -> None:
        """
        Start the assessment.
        :return: None
        """
        pass


    def end_assessment(self) -> None:
        """
        End the assessment.
        :return: None
        """
        self.remove_subscribers()
        self.check_requirements()


    def check_requirements(self) -> None:
        """
        Check the requirements.
        :return: None
        """
        pass

    
    def save_record(self, target: List, data: Any) -> None:
        """
        Save the data to the target list with a timestamp.
        :param target: List
        :param data: Any
        :return: None
        """
        target.append((self.get_time_elapsed(), data))

    
    def check_points(self, positions: List[Tuple], target: List[Tuple], ordered: bool = False, tolerance: float = 0.05) -> bool:
        """
        Check if certain points occur in a target record.
        :param positions: List[Tuple]
        :param tolerance: float
        :return: bool
        """
        
        if ordered:
            try:
                return ordered_points(positions, target, tolerance)
            except Exception as e:
                self.logger.error(e)
                return False
        else:
            try:
                return unordered_points(positions, target, tolerance)
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
                print(value, target, comp)
                return comparison_dict[comp]()

            else:
                raise ValueError("Invalid comparison operator.")
        
        else:
            raise ValueError("Invalid target.")
            
        return False
    

    def check_requirements(self, requirements: dict = {}) -> bool:

        """
        Check if the requirements are met.
        :param requirements: List[Tuple]
        :return: bool
        """
        # hardcoded requirements for test purposes

        # new test case is a rectangular path with different speeds and distances
        requirements = {
            "ordered_path": [(10, 5.5), (10, 10.5), (5, 10.5), (5, 5.5)], # should fail
            "unordered_path": [(10, 5.5), (5.5, 5.5), (5.5, 10.5), (10, 10.5)], # should fail
            "average_velocity": (0.1, '>'), # should fail
            "frequency": (60.0, '='), # should fail
            "execution_time": [(30, 40)] # should fail
        }

        for req, params in requirements.items():
            if req == "ordered_path":
                res = self.check_points(params, self.pose_record, ordered=True)
                self.logger.info(f"Result for ordered path verification: {res}")
                    
            elif req == "unordered_path":
                res = self.check_points(params, self.pose_record)
                self.logger.info(f"Result for unordered path verification: {res}")

            elif req == "average_velocity":
                res = self.check_value_params(value=get_average_value(self.cmd_vel_record), target=params[0], comp=params[1])
                self.logger.info(f"Result for average velocity verification: {res}")

            elif req == "frequency":
                #print(self.frequency_of_events(self.pose_record), params[0], params[1])
                res = self.check_value_params(value=frequency_of_events(self.cmd_vel_record), target=params[0], comp=params[1])
                self.logger.info(f"Result for frequency verification: {res}")
            
            elif req == "execution_time":
                res = self.check_value_params(value=self.total_execution_time, target=params[0], comp="=" if len(params) == 1 else params[1])
                self.logger.info(f"Result for execution time verification: {res}")
            
            else:
                raise ValueError(f"{req} - Invalid requirement.")
        
        return True
    

    def run(self) -> None:
        """
        Run the assessment.
        :return: None
        """
        self.global_events_monitor = rospy.Subscriber('global_events', GlobalEvents, self.handle_sub, queue_size=10)

        while not rospy.is_shutdown():
            self.rate.sleep()
        
    


class PoseAssessment(AssessmentObject):
    """
    Class to assess the pose of the turtle.
    """
    def __init__(self):
        super().__init__()
        self.curr_pose = Pose()
        self.distance = 0
        self.pose_record = []


    def create_subscribers(self) -> None:
        self.pose_sub = rospy.Subscriber('turtle1/pose', Pose, self.handle_sub, queue_size=10)
        self.subscribers.append(self.pose_sub)


    def handle_sub(self, data: Pose) -> None:
        self.save_record(self.pose_record, data)
        self.curr_pose = data
        self.update_distance_travelled()


    def update_distance_travelled(self) -> None:
        """
        Update the distance travelled by the turtle.
        :return: None
        """
        if len(self.pose_record) > 1:
            # unpack the tuples and calculate the distance between the last two poses
            curr_x, curr_y = self.pose_record[-1][1].x, self.pose_record[-1][1].y
            prev_x, prev_y = self.pose_record[-2][1].x, self.pose_record[-2][1].y
            self.distance += ((curr_x - prev_x) ** 2 + (curr_y - prev_y) ** 2) ** 0.5

    
    def start_assessment(self) -> None:
        """
        Start the assessment.
        :return: None
        """
        self.create_subscribers()




class CmdVelAssessment(AssessmentObject):
    """
    Class to assess the command velocity of the turtle.
    """
    def __init__(self):
        super().__init__()
        self.curr_vel = Twist()
        self.cmd_vel_record = []

    def create_subscribers(self) -> None:
        self.cmd_vel_sub = rospy.Subscriber('turtle1/cmd_vel', Twist, self.handle_sub, queue_size=10)
        self.subscribers.append(self.cmd_vel_sub)


    def handle_sub(self, data: Twist) -> None:
        self.save_record(self.cmd_vel_record, data)
        self.curr_vel = data


    def start_assessment(self) -> None:
        """
        Start the assessment.
        :return: None
        """
        self.start_time = rospy.get_time()
        self.cmd_vel_record = []
        self.cmd_vel_sub = rospy.Subscriber('turtle1/cmd_vel', Twist, self.handle_sub, queue_size=10)
        self.subscribers.append(self.cmd_vel_sub)