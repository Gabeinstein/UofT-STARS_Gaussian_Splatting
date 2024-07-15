#!/usr/bin/env python3

import rospy
import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

class dataAcquisitionService:
    def __init__(self, desired_link, camera_topic):
        rospy.init_node('dataAcquisitionServiceNode')

        self.listener = tf.TransformListener()
        self.link_name = rospy.get_param('~link_name', desired_link)
        self.image_topic = rospy.get_param('~image_topic', camera_topic)

        self.bridge = CvBridge()
        self.image_list = []
        self.data = []

        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.rate = rospy.Rate(10)
        self.angle_shots = [36, 25, 15, 12, 7, 4, 1]
        self.anles = [0, math.pi/20, math.pi*2/10, math.pi*3/20,math.pi*4/20,math.pi*5/20,
                      math.pi*6/20, math.pi*7/20, math.pi*8/20, math.pi*9/20, math.pi/2]
        self.angles_index = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_list.append(cv_image)
            if len(self.image_list) > 10:
                self.image_list.pop(0)
        except rospy.ROSInterruptException:
            pass
        
    def get_image_phi_random(self):
        camera_position_array = []
        for i in range(0,self.angle_shots[self.angles_index]):
            camera_position_array.append(np.random.uniform(-math.pi, math.pi))

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

    def run(self):
        while not rospy.is_shutdown():
            self.get_link_position()
            self.display_images()
            self.rate.sleep()

if __name__ == '__main__':
    myservice = dataAcquisitionService(desired_link='camera_link_optical', camera_topic='/panda/camera1/image_raw')
    myservice.run()