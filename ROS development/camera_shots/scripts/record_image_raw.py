#! /usr/bin/env python3

import rosbag
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
import csv

def extract_data(bag_file, topics):
    image_data = []
    tf_data = []
    with rosbag.Bag(bag_file, 'r') as bag:
        msg = None
        for topic, msg, t in bag.read_messages():
            if topic == 'tf':
                
                tf_data.append(msg.transforms)
                print(msg.transforms)
    return image_data, tf_data

bag_file = '/home/starslab_go/projects/starlab_ws/src/camera_shots/recordings/0/1_05_-1_0_-1_05_-1.bag'
topics = ['tf' '/panda/camera1/image_raw']

recorded_data = extract_data(bag_file, topics)
