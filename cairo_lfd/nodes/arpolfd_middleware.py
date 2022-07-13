#!/usr/bin/env python3

import rospy

from cairo_lfd.middleware.ar_middleware import ARPOLfDMiddleware

from geometry_msgs.msg import Quaternion, Vector3


def main():
    rospy.init_node("arpolfd_middleware")
    mm = ARPOLfDMiddleware(ar_pos_transform=Vector3(0.975, -0.09, -0.27), ar_quat_transform=Quaternion(0.0, 0.0, 1.0, 0.0), joint_configuration_topic="arpo_lfd/joint_configuration", playback_cmd_topic="arpo_lfd/trajectory_playback_cmd", joint_trajectory_topic="arpo_lfd/joint_trajectory")
    rospy.spin()

  
if __name__ == '__main__':
    main()
