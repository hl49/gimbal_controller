#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState


class Gimbal:
    # def shutdown():

    def __init__(self):

        # Set variables
        # Node frequency
        self.frequency = 1.0/10.0 
        self.taget = []
        self

        # Init ROS node
        rospy.init_node("gimbal_controller")


        # Create topics 

        # self.sub_target = rospy.Subscriber("/Imu_data",
        #         Imu,
        #         self._get_target_callback,
        #         queue_size=1)
    
        # self.sub_imu = rospy.Subscriber("/Imu_data",
        #         Imu,
        #         self._get_imu_callback,
        #         queue_size=1)


        self.servo_pub = rospy.Publisher("/desired_joint_states",
                    JointState,
                    queue_size=1)

    def get_target_callback(self):
        pass

    def get_imu_callback(self):
        pass

    def publish_servos_ref(self):
        pass

    def run(self, event=None):

        print("hey!")
        # print(type(event))

        # calc pid

        # if there is not target position, set 0. 

        # set servos



if __name__ == "__main__":

    gimbal = Gimbal()

    rospy.Timer(rospy.Duration(gimbal.frequency), gimbal.run)

    rospy.spin()

    # try:
    #     gimbal.run()

    # except rospy.ROSInterruptException:
    #     pass
    #     # shutdown_controller()