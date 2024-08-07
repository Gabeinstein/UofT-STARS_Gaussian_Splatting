#!/usr/bin/env python3

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()

    try:
        rospy.loginfo("Waiting for tf transform between '/base_link' and '/desired_link'")
        listener.waitForTransform('/world', '/camera_link_optical', rospy.Time(), rospy.Duration(4.0))
        rospy.loginfo("Got transform!")

        rate = rospy.Rate(1.0)  # 1 Hz
        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform('/world', '/camera_link_optical', rospy.Time(0))
                rospy.loginfo(f"Translation: {trans}")
                rospy.loginfo(f"Rotation: {rot}")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"Failed to get transformation: {e}")

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
