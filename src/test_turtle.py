#!/usr/bin/env python3

from turtle_movement.TurtleSquare import TurtleSquare
from turtle_movement.TurtleRectangle import TurtleRectangle
from rospy.exceptions import ROSInterruptException

if __name__ == '__main__':
    try:
        turtle_square = TurtleSquare()
        turtle_square.move()

    except ROSInterruptException:
        print("Interrupting execution...")
        raise