#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

referenced from tizianofiorenzani/ros_tutorials
url: https://github.com/tizianofiorenzani/ros_tutorials

Subscribes to 
    /darknet_ros/bounding_boxes
    
Publishes commands to 
    /dkcar/control/cmd_vel    

"""
import time
import rospy
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
from rospy.topics import Message

K_LAT_DIST_TO_STEER = -2.0
K_LAT_DIST_TO_THROTTLE = 0.2

#global variable for keeping this value to next traffic signal
throttle_action = 0.0

def saturate(value, min, max):
    if value <= min:
        return min
    elif value >= max:
        return max
    else:
        return value

class ChaseObject:
    def __init__(self):

        self.blob_x = 0.0
        self.blob_y = 0.0
        self._time_detected = 0.0

        self.sub_center = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.update_object)
        rospy.loginfo("Subscribers set")

        self.pub_twist = rospy.Publisher("/dkcar/control/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")

        self._message = Twist()

        self._time_steer = 0
        self._steer_sign_prev = 0

    @property
    def is_detected(self):
        return time.time() - self._time_detected < 1.0

    def update_object(self, message):
        for box in message.bounding_boxes:
            #
            #yolov4-tiny, 416x416
            if box.Class == "left":
                self.blob_x = 0.5
                self.blob_y = 1.0
            elif box.Class == "right":
                self.blob_x = -0.5
                self.blob_y = 1.0
            elif box.Class == "go":    
                self.blob_x = 0.0
                self.blob_y = 1.0
            elif box.Class == "stop":     
                self.blob_x = 0.0
                self.blob_y = 0.0

            self._time_detected = time.time()
            #rospy.loginfo("object detected: %.2f  %.2f "%(self.blob_x, self.blob_y))
            rospy.loginfo(
                    "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {} Class: {}".format
                    (box.xmin, box.xmax, box.ymin, box.ymax, box.Class) )
            
    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        Steer will be added to the commanded throttle
        throttle will be multiplied by the commanded throttle
        """
        steer_action = 0.0
        global throttle_action

        if self.is_detected:
            # --- Apply steering, proportional to how close is the object
            steer_action = -K_LAT_DIST_TO_STEER * self.blob_x
            steer_action = saturate(steer_action, -1.5, 1.5)            
            #if object is detected, go forward with 20% power
            throttle_action = K_LAT_DIST_TO_THROTTLE*self.blob_y
            rospy.loginfo("is _detected, Steering = %3.1f Throttle = %3.1f" % (steer_action, throttle_action))

        return (steer_action)

    def run(self):

        # --- Set the control rate
        global throttle_action
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            # -- Get the control action
            steer_action = self.get_control_action()
            rospy.loginfo("RUN, Steering = %3.1f Throttle = %3.1f" % (steer_action, throttle_action))

            # -- update the message
            self._message.linear.x = throttle_action
            self._message.angular.z = steer_action

            # -- publish it
            self.pub_twist.publish(self._message)

            rate.sleep()


if __name__ == "__main__":

    rospy.init_node("chase_gostop_yolo")

    chase_ball = ChaseObject()
    chase_ball.run()
