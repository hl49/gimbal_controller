#!/usr/bin/env python

# from atexit import unregister
from logging import shutdown
from ntpath import join
import rospy
import numpy as np
import math
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
        self.ini_tilt = 0.0
        self.ini_roll = 0.0
        self.count_init_servo_pos = 0
        self.num_samples_init = 20
        self.ini_tilt_arr = np.zeros([self.num_samples_init, 1])
        self.ini_roll_arr = np.zeros([self.num_samples_init, 1])

        # self.max_tilt = math.radians(65)
        # self.max_roll = math.radians(45)

        # Init ROS node
        rospy.init_node("gimbal_controller")


        # Create topics 

        # self.sub_target = rospy.Subscriber("/Imu_data",
        #         Imu,
        #         self._get_target_callback,
        #         queue_size=1)

        self.servo_pub = rospy.Publisher("/desired_joint_states",
                    JointState,
                    queue_size = 1)

        self.sub_joint_states = rospy.Subscriber("/joint_states",
                    JointState,
                    self.get_ini_joint_states_callback,
                    queue_size = 1)

        self.sub_imu = rospy.Subscriber("/pixhawk_rpy",
                    Float32MultiArray,
                    self.get_imu_callback,
                    queue_size=1)


    def get_ini_joint_states_callback(self, data):
        if self.count_init_servo_pos < self.num_samples_init:
            joints = dict(zip(data.name, data.position))
            self.ini_tilt_arr[self.count_init_servo_pos, 0] = joints['tilt']
            self.ini_roll_arr[self.count_init_servo_pos, 0] = joints['roll']
            self.count_init_servo_pos += 1
        else:
            self.ini_tilt = self.ini_tilt_arr.mean()
            self.ini_roll = self.ini_roll_arr.mean()
            self.sub_joint_states.unregister()
        

    def get_imu_callback(self, data):
        # print(data.data)
        self.tilt = round(data.data[0],2) * -1.0
        self.roll = round(data.data[1],2) * -1.0

        # if self.tilt > - self.max_tilt and self.tilt <

    def publish_servos_ref(self):

        servo_data = JointState()

        servo_data.header.stamp = rospy.Time.now()
        servo_data.header.frame_id = ''
        servo_data.header.seq = self.servo_data_seq_counter

        servo_data.name = ["tilt", "roll"]
        servo_data.position = [self.tilt, self.roll]

        self.servo_data_seq_counter =+ 1
        self.servo_pub.publish(servo_data)


    def stabilize(self, event=None):

        # if there is not target position, set 0. 

        # Controller
        
        # set servos
        self.publish_servos_ref()


if __name__ == "__main__":


    gimbal = Gimbal()

    rospy.Timer(rospy.Duration(gimbal.frequency), gimbal.stabilize)

    rospy.spin()

    
