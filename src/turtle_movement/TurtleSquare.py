#!/usr/bin/env python3

from turtle_movement.TurtleShape import TurtleShape

class TurtleSquare(TurtleShape):
    def __init__(self):
        super().__init__()

    def move(self, speed=1, length=1):
        print("Starting position: ", self.pose.x, self.pose.y)
        for _ in range(4):
            self.move_forward(speed=speed, distance=length)
            print("Position after side", _ + 1, ":", self.pose.x, self.pose.y)
            self.rotate_half_pi()
            print("Position after rotation", _ + 1, ":", self.pose.x, self.pose.y)