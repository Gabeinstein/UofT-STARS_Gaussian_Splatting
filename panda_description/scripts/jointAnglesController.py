#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def publish_joint_angles():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher', anonymous=True)
    rate = rospy.Rate(10)

    joint_state = JointState()
    joint_state.header = Header()
    joint_state.name = [
        'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
        'panda_joint5', 'panda_joint6', 'panda_joint7',
        'panda_finger_joint1', 'panda_finger_joint2']

    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position = [0, 0.5, 0, -0.5, 0, 1, 1, 0, 0]
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_angles()
    except rospy.ROSInterruptException:
        pass