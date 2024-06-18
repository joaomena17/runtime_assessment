from TurtleSquare import TurtleSquare
from TurtleRectangle import TurtleRectangle
from rospy.exceptions import ROSInterruptException

if __name__ == '__main__':
    try:
        turtle_square = TurtleSquare()
        turtle_square.move(speed=1, length=5)

    except ROSInterruptException:
        print("Interrupting execution...")
        raise