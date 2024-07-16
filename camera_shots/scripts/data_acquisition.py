#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
import argparse
import sys
from std_msgs.msg import Float64


class dataAcquisitionService:
    def __init__(self, desired_link, camera_topic):
        

        parser = argparse.ArgumentParser()
        parser.add_argument('node_name', type=str,  help='node_name')
        parser.add_argument('mode', type=int,  help='display (1) or screenshot (0)')
        parser.add_argument('theta', type=int, help='int value (0-10) camera angle => -pi*input/20')
        parser.add_argument('shots', type=int, help='Number of shots')
        self.args = parser.parse_args()
        
        rospy.init_node(self.args.node_name)

        if (self.args.mode):
            self.rate = rospy.Rate(10)
        

        self.listener = tf.TransformListener()
        self.link_name = rospy.get_param('~link_name', desired_link)
        self.image_topic = rospy.get_param('~image_topic', camera_topic)

        self.bridge = CvBridge()
        self.image_list = []

        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.camera_phi_pub = rospy.Publisher("/panda/camera_phi_joint_position_controller/command",Float64,queue_size=10)
        self.camera_theta_pub = rospy.Publisher("/panda/camera_theta_joint_position_controller/command",Float64,queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_list.append(cv_image)
            if len(self.image_list) > 10:
                self.image_list.pop(0)
        except rospy.ROSInterruptException:
            pass
        
    def set_phi_camera_random(self):
        angle = np.random.uniform(-math.pi, math.pi)
        self.camera_phi_pub.publish(angle)

    def get_link_position(self):
        try:
            self.listener.waitForTransform('/world', self.link_name, rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = self.listener.lookupTransform('/world', self.link_name, rospy.Time(0))
            rospy.loginfo(f"Position: {trans}")
            rospy.loginfo(f"Rotation: {rot}")
            return trans, rot
        except rospy.ROSInterruptException:
            pass

    def display_images(self):
        if self.image_list:
            cv2.imshow('Camera Image', self.image_list[-1])
            cv2.waitKey(1)

    def set_theta_camera_angle(self):
        angle = -math.pi*(self.args.theta)/20
        self.camera_theta_pub.publish(angle)
        

    def run(self):
        while not rospy.is_shutdown():
            if (self.args.mode):
                self.get_link_position()
                self.display_images()
                self.rate.sleep()
            else:
                for i in range(self.args.shots + 2):
                    self.set_theta_camera_angle()
                    self.set_phi_camera_random()
                    rospy.sleep(4)
                    print(f'shot {i}!')

                sys.exit(0)

if __name__ == '__main__':
    myservice = dataAcquisitionService(desired_link='camera_link_optical', camera_topic='/panda/camera1/image_raw')
    myservice.run()