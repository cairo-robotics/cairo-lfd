#!/usr/bin/env python3

import rospy

from cairo_lfd.middleware.transform_middleware import FixedTransform

from geometry_msgs.msg import Quaternion, Vector3


def main():
    rospy.init_node("vrpn_middleware")
    fixed_transform = FixedTransform(parent_frame="world", child_frame="optitrack_world", translation=Vector3(0, 0, 0), rotation=Quaternion( 0, 0, 0, 1))
    rospy.spin()

  
if __name__ == '__main__':
    main()
