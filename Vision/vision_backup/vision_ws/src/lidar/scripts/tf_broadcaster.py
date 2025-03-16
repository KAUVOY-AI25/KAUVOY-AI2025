#!/usr/bin/env python

import rospy
import tf

def broadcast_transform():
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)  # 10Hz

    while not rospy.is_shutdown():
        br.sendTransform(
            (0, 0, 0),  # Translation (x, y, z) -> 필요 시 조정
            tf.transformations.quaternion_from_euler(0, 0, 0),  # Rotation (r, p, y)
            rospy.Time.now(),
            "map",  # Target Frame (Waypoint 기준)
            "velodyne"  # Source Frame (LiDAR 기준)
        )
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass

