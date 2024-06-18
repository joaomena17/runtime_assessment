#!/usr/bin/env python3

from TurtleShape import TurtleShape

class TurtleSquare(TurtleShape):
    def __init__(self):
        super().__init__()
    
    def move(self, speed=1, length=5):
        print("Starting position: ", self.pose.x, self.pose.y)
        for _ in range(2):
            self.move_forward(speed=speed, distance=length)
            print("Position after side", _ + 1, ":", self.pose.x, self.pose.y)
            self.rotate_half_pi(speed=0.25)
            print("Position after rotation", _ + 1, ":", self.pose.x, self.pose.y)
            self.move_forward(speed=speed, distance=2*length)
            print("Position after side", _ + 2, ":", self.pose.x, self.pose.y)
            self.rotate_half_pi(speed=0.25)
            print("Position after rotation", _ + 2, ":", self.pose.x, self.pose.y)