#!/usr/bin/env python3

import rospy

from cairo_lfd.middleware.vrpn_middleware import VRPNFixedTransform

from geometry_msgs.msg import Quaternion, Vector3


def main():
    rospy.init_node("vrpn_middleware")
    vrpn_transform = VRPNFixedTransform(vrpn_world_frame="optitrack_world", global_world_frame="world", translation=Vector3(0, 0, 0), rotation=Quaternion(0, 0, 0, 1))
    rospy.spin()

  
if __name__ == '__main__':
    main()
