#!/usr/bin/env python3

from turtle_movement.TurtleShape import TurtleShape
from std_msgs.msg import String
import rospy

class TurtleSquare(TurtleShape):
    def __init__(self):
        super().__init__()
        self.checkpoint_publisher = rospy.Publisher("/turtle1/checkpoint", String, queue_size=10)


    def pub_checkpoint(self, i):
        data = f"reached {i}"
        self.checkpoint_publisher.publish(data)
        print(f"Published checkpoint {i} with data {data}")


    def move(self, speed=1, length=5): 
        print("Starting position: ", self.pose.x, self.pose.y)
        c = 1
        for _ in range(2):
            self.move_forward(speed=speed, distance=length)
            print("Position after side", c, ":", self.pose.x, self.pose.y)
            self.pub_checkpoint(c)
            c += 1

            self.rotate_half_pi(speed=0.5)
            print("Position after rotation", c, ":", self.pose.x, self.pose.y)
            
            self.move_forward(speed=speed, distance=length)
            print("Position after side", c, ":", self.pose.x, self.pose.y)
            self.pub_checkpoint(c)
            c += 1

            self.rotate_half_pi(speed=0.5)
            print("Position after rotation", c, ":", self.pose.x, self.pose.y)