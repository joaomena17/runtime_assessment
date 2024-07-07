#!/usr/bin/env python3

from turtle_movement.TurtleShape import TurtleShape
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped  # Import TransformStamped message type
from tf2_msgs.msg import TFMessage  # Import TFMessage
import rospy
import tf.transformations  # Import transformations module

class TurtleSquare(TurtleShape):
    def __init__(self):
        super().__init__()
        self.checkpoint_publisher = rospy.Publisher("/turtle1/checkpoint", String, queue_size=10)
        self.transform_publisher = rospy.Publisher("/tf", TFMessage, queue_size=10)  # Initialize TFMessage publisher

    def pub_checkpoint(self, i):
        data = f"reached {i}"
        self.checkpoint_publisher.publish(data)
        print(f"Published checkpoint {i} with data {data}")

    def publish_transform(self):
        # Create and publish the transform message
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = "turtle"
        transform.transform.translation.x = self.pose.x
        transform.transform.translation.y = self.pose.y
        transform.transform.translation.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        # Create a TFMessage and add the transform
        tf_message = TFMessage([transform])
        self.transform_publisher.publish(tf_message)
        print(f"Published transform: {tf_message}")

    def move(self, speed=1, length=5): 
        print("Starting position: ", self.pose.x, self.pose.y)
        c = 1
        for _ in range(2):
            self.move_forward(speed=speed, distance=length)
            print("Position after side", c, ":", self.pose.x, self.pose.y)
            self.pub_checkpoint(c)
            self.publish_transform()  # Publish transform after moving forward
            c += 1

            self.rotate_half_pi(speed=0.5)
            print("Position after rotation", c, ":", self.pose.x, self.pose.y)
            self.publish_transform()  # Publish transform after rotation
            
            self.move_forward(speed=speed, distance=length)
            print("Position after side", c, ":", self.pose.x, self.pose.y)
            self.pub_checkpoint(c)
            self.publish_transform()  # Publish transform after moving forward
            c += 1

            self.rotate_half_pi(speed=0.5)
            print("Position after rotation", c, ":", self.pose.x, self.pose.y)
            self.publish_transform()  # Publish transform after rotation