#!/usr/bin/env python3

import os
import yaml
import rospy
import rosnode
import logging
from utils import *
from datetime import datetime
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from typing import List, Tuple, Any, Set, Union




class RuntimeAssessment:
    """
    Class to assess the runtime behavior of a ROS application.
    """
    def __init__(self, target_node: str, log_path: str = "/home/joaomena/catkin_ws/src/runtime_assessment/src/log"):
        # Assessment variables
        self.node = rospy.init_node('runtime_assessment')
        self.target_node = target_node
        self.previous_nodes: Set[str] = set()
        self.rate = rospy.Rate(10)
        self.is_paused = False
        self.is_running = False

        # Logger setup
        self.logger = logging.getLogger(f"RuntimeAssessment.{self.target_node}")

        current_date = datetime.now().strftime("%Y_%m_%d")
        log_dir = os.path.join(log_path, f"{self.target_node.split('/')[1]}_{current_date}")
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

    
    def create_subscribers(self) -> None:
        """
        Create the subscribers.
        :return: None
        """
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.record_pose, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/turtle1/cmd_vel', Twist, self.record_cmd_vel, queue_size=10)
        self.logger.debug("Subscribers created.")

    
    def remove_subscribers(self):
        """
        Remove the subscribers.
        :return: None
        """
        self.pose_sub.unregister()
        self.cmd_vel_sub.unregister()
        self.logger.debug("Subscribers removed.")
    

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
                        self.end_assessment()
                    else:
                        self.logger.info("Assessment paused.")
                        rospy.sleep(1.0)

                else:
                    if not target_running:
                        self.logger.info("Target node removed. Finishing assessment...")
                        self.end_assessment()
                    else:
                        self.rate.sleep()   
                
            else:
                if target_running:
                    self.logger.info("Target node started. Initializing assessment...")
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
            self.remove_subscribers()
            self.logger.info("Assessment paused.")


    def resume(self):
        """
        Resume the assessment.
        :return: None
        """
        if self.is_paused:
            self.is_paused = False
            self.is_running = True
            self.create_subscribers()
            self.logger.info("Assessment resumed.")


    def start_assessment(self) -> None:
        """
        Start the assessment.
        :return: None
        """
        self.start_time = rospy.get_time()
        self.pose_record = []
        self.cmd_vel_record = []
        self.pose = Pose()
        self.vel = Twist()
        self.distance = 0
        self.is_paused = False
        self.is_running = True
        self.create_subscribers()
        self.logger.info("Assessment started.")


    def end_assessment(self) -> None:
        """
        End the assessment.
        :return: None
        """
        rospy.signal_shutdown("Assessment stopped.")
        self.is_running = False
        self.is_paused = False
        self.remove_subscribers()
        self.total_execution_time = self.get_time_elapsed()
        self.logger.info("Assessment ended.")
        self.logger.info(f"Execution time: {self.total_execution_time} seconds.")
        self.logger.info(f"Distance travelled: {self.distance} units.")
        self.logger.info(f"Average velocity: {get_average_value(self.cmd_vel_record)} units/s.")
        self.logger.info(f"Frequency of events: {frequency_of_events(self.cmd_vel_record)} seconds.")
        self.check_requirements()
        self.logger.info("----------------- END OF ASSESSMENT -----------------\n")


    def save_record(self, target: List, data: Any) -> None:
        """
        Save the data to the target list with a timestamp.
        :param target: List
        :param data: Any
        :return: None
        """
        target.append((self.get_time_elapsed(), data))

    
    def get_time_elapsed(self) -> float:
        """
        Get the time elapsed since the assessment started.
        :return: float
        """
        return rospy.get_time() - self.start_time

    
    def record_pose(self, data: Pose) -> None:
        """
        Record the pose of the turtle.
        :param data: Pose
        :return: None
        """
        self.save_record(self.pose_record, data)
        self.update_distance_travelled()


    def record_cmd_vel(self, data: Twist) -> None:
        """
        Record the command velocity of the turtle.
        :param data: Twist
        :return: None
        """
        self.vel = data
        self.save_record(self.cmd_vel_record, data)


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


    def check_unordered_path_in_poses(self, positions: List[Tuple], target: List[Tuple], ordered: bool = False, tolerance: float = 0.05) -> bool:
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
                res = self.check_ordered_path_in_poses(params)
                self.logger.info(f"Result for ordered path verification: {res}")
                    
            elif req == "unordered_path":
                res = self.check_unordered_path_in_poses(params)
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