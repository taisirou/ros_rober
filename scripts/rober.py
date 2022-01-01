#!/usr/bin/python3
import time
import rospy
from rospy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from MotorDriver.PCA9685 import PCA9685

Dir = [
    'forward',
    'backward',
]
pwm = PCA9685(0x40, debug=False)
pwm.setPWMFreq(50)

class MotorDriver():
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return
        if(motor == 0):
            pwm.setDutycycle(self.PWMA, speed)
            if(index == Dir[0]):
                pwm.setLevel(self.AIN1, 0)
                pwm.setLevel(self.AIN2, 1)
            else:
                pwm.setLevel(self.AIN1, 1)
                pwm.setLevel(self.AIN2, 0)
        else:
            pwm.setDutycycle(self.PWMB, speed)
            if(index == Dir[0]):
                pwm.setLevel(self.BIN1, 0)
                pwm.setLevel(self.BIN2, 1)
            else:
                pwm.setLevel(self.BIN1, 1)
                pwm.setLevel(self.BIN2, 0)

    def MotorStop(self, motor):
        if (motor == 0):
            pwm.setDutycycle(self.PWMA, 0)
        else:
            pwm.setDutycycle(self.PWMB, 0)

class Rober:
    def __init__(self):
        rospy.init_node('rober', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.motor = MotorDriver()
        self.speed = 0
        self.motor.MotorRun(0, Dir[0], self.speed)
        self.motor.MotorRun(1, Dir[0], self.speed)
        self.pub = rospy.Publisher('scan', Range, queue_size=10)
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.callback, queue_size=10)

    def callback(self, data):
        self.speed = data.linear.x * 100
        self.angle = data.angular.z * 100
        rospy.loginfo('set linear.x = {}, linear.z = {}'.format(self.speed, self.angle))
 
        if self.speed > 0:
            self.motor.MotorRun(0, Dir[0], self.speed)
            self.motor.MotorRun(1, Dir[0], self.speed)
        elif self.speed == 0:
            self.motor.MotorStop(0)
            self.motor.MotorStop(1)
        else:
            self.motor.MotorRun(0, Dir[1], -self.speed)
            self.motor.MotorRun(1, Dir[1], -self.speed)
        
        if self.angle > 0:
            self.motor.MotorRun(1, Dir[0], self.angle)
            self.motor.MotorRun(0, Dir[1], self.angle)
        elif self.angle < 0:
            self.motor.MotorRun(1, Dir[1], -self.angle)
            self.motor.MotorRun(0, Dir[0], -self.angle)
    
    def shutdown(self):
        rospy.loginfo("Stop")
        self.motor.MotorStop(0)
        self.motor.MotorStop(1)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rober = Rober()
        rospy.loginfo('start')
        rospy.spin()
    except ROSInterruptException:
        pass
