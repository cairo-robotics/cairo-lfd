#!/usr/bin/env python3

import rospy

from cairo_lfd.middleware.ar_middleware import ARPOLfDMiddleware
from cairo_lfd.middleware.transform_middleware import ARVRFixedTransform

from geometry_msgs.msg import Quaternion, Vector3


def main():
    rospy.init_node("arpolfd_middleware")
    transform_manager = ARVRFixedTransform("world", "hololens", Vector3(0.975, -0.09, -0.27), Quaternion(0.0, 0.0, 1.0, 0.0), [[0, 0, 1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, -1]])
    mm = ARPOLfDMiddleware(transform_manager, joint_configuration_topic="arpo_lfd/joint_configuration", playback_cmd_topic="arpo_lfd/trajectory_playback_cmd", joint_trajectory_topic="arpo_lfd/joint_trajectory")
    rospy.spin()

  
if __name__ == '__main__':
    main()
