#! /usr/bin/python3
import rospy
from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Contorller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.linear_x = 0
        self.angular_zr = 0
        self.angular_zl = 0
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.on_shutdown(self.shutdown_callback)
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)
    
    def joy_callback(self, joy_msg):
        # Get Joy Event
        self.linear_x = joy_msg.axes[1]
        self.angular_zr = joy_msg.buttons[1]
        self.angular_zl = joy_msg.buttons[3]

    def timer_callback(self, event):
        twist = Twist()
        twist.linear.x = int(self.linear_x) * 0.3
        twist.angular.z = int(self.angular_zl - self.angular_zr) * 0.3
        rate = rospy.Rate(10)
        for i in range(5):
            self.pub.publish(twist)
            rospy.loginfo("linear_x: %f, angular_z: %f", twist.linear.x, twist.angular.z)
            rate.sleep()

    def shutdown_callback(self):
        rospy.loginfo("Stop")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        rospy.loginfo("Start")
        Contorller()
        rospy.spin()
    except ROSInterruptException:
        pass
