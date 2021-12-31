#!/usr/bin/python3
import time
import rospy
from rospy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist

class Contorller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.is_forward = True
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        # Twistの初期化
        twiet = Twist()
        twiet.linear.x = 0
        twiet.angular.z = 0
        self.pub.publish(twiet)

    def timer_callback(self, event):
        # Twistの初期化
        twiet = Twist()
        if self.is_forward:
            twiet.linear.x = 0.5
        else:
            twiet.linear.x = -0.5
        self.pub.publish(twiet)
        rate = rospy.Rate(10)
        rate.sleep()
        twiet.linear.x = 0
        self.pub.publish(twiet)
        self.is_forward = not self.is_forward

if __name__ == '__main__':
    try:
        Contorller()
        rospy.spin()
    except ROSInterruptException:
        pass
