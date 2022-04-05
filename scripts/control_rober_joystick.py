#! /usr/bin/env python3
import rospy
from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Contorller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.on_shutdown(self.shutdown_callback)
    
    def joy_callback(self, joy_msg):
        axes = joy_msg.axes
        buttons = joy_msg.buttons
        rospy.loginfo("axes: %s", axes)
        rospy.loginfo("buttons: %s", buttons)

    def shutdown_callback(self):
        rospy.loginfo("Stop")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Contorller()
        rospy.spin()
    except ROSInterruptException:
        pass
