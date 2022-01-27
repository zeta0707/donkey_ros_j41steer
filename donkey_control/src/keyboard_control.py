#!/usr/bin/env python

"""
Node for control PCA9685 using teleop_twist_keyboard msg 
referenced from donekycar
url : https://github.com/autorope/donkeycar/blob/dev/donkeycar/parts/actuator.py
"""

import time
import rospy
from threading import Thread
from geometry_msgs.msg import Twist

STEER_CENTER = 380
STEER_STEP = -10
STEER_LIMIT = 100
SPEED_STEP = 1024

speed_pulse = 0
steering_pulse = STEER_CENTER

class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """

    def __init__(
           self, channel, address, frequency=60, busnum=None, init_delay=0.1
    ):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685

        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C

            # replace the get_bus function with our own
            def get_bus():
                return busnum

            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay)  # "Tamiya TBLE-02" makes a little leap otherwise

        self.pulse = STEER_CENTER
        self.prev_pulse = STEER_CENTER
        self.running = True

    def set_pwm(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        pulse_diff = pulse - self.prev_pulse

        if abs(pulse_diff) > 40:
            if pulse_diff > 0:
                pulse += 0.7 * pulse_diff
            else:
                pulse -= 0.7 * pulse_diff

        self.set_pwm(pulse)
        self.prev_pulse = pulse

    def set_pulse(self, pulse):
        self.pulse = pulse

    def update(self):
        while self.running:
            self.set_pulse(self.pulse)

class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    def __init__(self, controller=None,
                       max_pulse=4095,
                       min_pulse=-4095,
                       zero_pulse=0):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        print("Init ESC")
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)


    def run(self, throttle):
        pulse = int(throttle)
        if throttle > 0:
            self.controller.pwm.set_pwm(self.controller.channel,0,pulse)
            self.controller.pwm.set_pwm(self.controller.channel+1,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+2,0,4095)
            self.controller.pwm.set_pwm(self.controller.channel+3,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+4,0,pulse)
            self.controller.pwm.set_pwm(self.controller.channel+7,0,pulse)
            self.controller.pwm.set_pwm(self.controller.channel+6,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+5,0,4095)
        else:
            self.controller.pwm.set_pwm(self.controller.channel,0,-pulse)
            self.controller.pwm.set_pwm(self.controller.channel+2,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+1,0,4095)
            self.controller.pwm.set_pwm(self.controller.channel+3,0,-pulse)
            self.controller.pwm.set_pwm(self.controller.channel+4,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+7,0,-pulse)
            self.controller.pwm.set_pwm(self.controller.channel+5,0,0)
            self.controller.pwm.set_pwm(self.controller.channel+6,0,4095)

    def shutdown(self):
        self.run(0) #stop vehicle

class Vehicle(object):
    def __init__(self, name="donkey_ros"):
        
        self._steering_servo = PCA9685(channel=0, address=0x40, busnum=1)
        rospy.loginfo("Steering Controller Awaked!!")

        throttle_controller = PCA9685(channel=0, address=0x60, busnum=1)
        self._throttle = PWMThrottle(controller=throttle_controller, max_pulse=4095, zero_pulse=0, min_pulse=-4095)
        rospy.loginfo("Throttle Controller Awaked!!") 
        
        self._name = name
        self._teleop_sub = rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.keyboard_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.loginfo("Keyboard Subscriber Awaked!! Waiting for keyboard...")

    def keyboard_callback(self, msg):

        global speed_pulse
        global steering_pulse
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        speed_pulse += msg.linear.x*SPEED_STEP

        if speed_pulse > 4095 :
           speed_pulse = 4095
        if speed_pulse < -4095 :
           speed_pulse = -4095       

        steering_pulse += msg.angular.z*STEER_STEP
        if steering_pulse > (STEER_CENTER + STEER_LIMIT) :
           steering_pulse = STEER_CENTER + STEER_LIMIT
        if steering_pulse < (STEER_CENTER - STEER_LIMIT) :
           steering_pulse = STEER_CENTER - STEER_LIMIT

        print(
            "speed_pulse : "
            + str(speed_pulse)
            + " / "
            + "steering_pulse : "
            + str(steering_pulse)
        )

        self._throttle.run(speed_pulse)
        self._steering_servo.run(steering_pulse)


if __name__ == "__main__":

    rospy.init_node("donkey_control")
    myCar = Vehicle("donkey_ros")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
