#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleSquare:
    def __init__(self):
        rospy.init_node('snake_game')
        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose = Pose()
        self.rate = rospy.Rate(60)
        self.direction = Twist()
    
    def update_pose(self, data):
        self.pose = data

    def move_forward(self, speed=1, distance=5):
        self.direction.linear.x = speed
        time_to_move = distance / speed
        move_time = rospy.Time.now()
        
        while rospy.Time.now() - move_time < rospy.Duration(time_to_move):
            self.pub.publish(self.direction)
            self.rate.sleep()
        
        self.direction.linear.x = 0.0
        self.pub.publish(self.direction)
    
    def rotate_half_pi(self):
        target_angle = self.pose.theta + math.pi/2

        if target_angle > math.pi:
            target_angle -= 2 * math.pi

        while abs(target_angle - self.pose.theta) > 0.001:
            self.direction.linear.x = 0.0

            if abs(target_angle - self.pose.theta) <= 0.05:
                self.direction.angular.z = 0.05
            elif abs(target_angle - self.pose.theta) <= 0.05:
                self.direction.angular.z = 0.1
            else:
                self.direction.angular.z = 0.5

            self.pub.publish(self.direction)
            self.rate.sleep()
        
        self.direction.angular.z = 0.0
        self.pub.publish(self.direction)


    def move(self, speed=1, length=1):
        pass