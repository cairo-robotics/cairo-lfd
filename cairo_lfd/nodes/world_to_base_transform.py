#!/usr/bin/env python3

import rospy

from cairo_lfd.middleware.transform_middleware import FixedTransform

from geometry_msgs.msg import Quaternion, Vector3


def main():
    rospy.init_node("world_to_base_transform")
    _ = FixedTransform(parent_frame="world", child_frame="base", translation=[0, 0, 0], rotation=[0, 0, 0, 1])    
    rospy.spin()

  
if __name__ == '__main__':
    main()
