#!/usr/bin/env python

import rospy
import numpy as np
import rostopic
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class Gimbal:
    # def shutdown():

    def __init__(self):

        # Set variables
        # Node frequency
        self.frequency = 1.0/10.0 
        self.taget = []
        self.servo_data_seq_counter = 0
        self.tilt = 0.0
        self.roll = 0.0

        # Init ROS node
        rospy.init_node("gimbal_controller")


        # Create topics 

        # self.sub_target = rospy.Subscriber("/Imu_data",
        #         Imu,
        #         self._get_target_callback,
        #         queue_size=1)

        self.servo_pub = rospy.Publisher("/desired_joint_states",
                    JointState,
                    queue_size=1)

        self.sub_imu = rospy.Subscriber("/pixhawk_rpy",
                Float32MultiArray,
                self._get_imu_callback,
                queue_size=1)


    def get_target_callback(self):
        pass

    def _get_imu_callback(self, data):
        # print(data.data)
        self.tilt = round(data.data[0],2) * -1.0
        self.roll = round(data.data[1],2) * -1.0

    def publish_servos_ref(self):

        servo_data = JointState()

        servo_data.header.stamp = rospy.Time.now()
        servo_data.header.frame_id = ''
        servo_data.header.seq = self.servo_data_seq_counter

        servo_data.name = ["tilt", "roll"]
        servo_data.position = [self.tilt, self.roll]

        self.servo_data_seq_counter =+ 1
        self.servo_pub.publish(servo_data)


    def run(self, event=None):

        # if there is not target position, set 0. 

        # set servos
        self.publish_servos_ref()
        


if __name__ == "__main__":


    gimbal = Gimbal()

    rospy.Timer(rospy.Duration(gimbal.frequency), gimbal.run)

    rospy.spin()

    
