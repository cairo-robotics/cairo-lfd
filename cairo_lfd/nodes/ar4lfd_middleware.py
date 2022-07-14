#!/usr/bin/env python3

import rospy

from cairo_lfd.middleware.ar_middleware import AR4LfDMiddleware
from cairo_lfd.middleware.transform_middleware import ARVRFixedTransform

from geometry_msgs.msg import Quaternion, Vector3

def main():
    rospy.init_node("ar4lfd_middleware")
    transform_manager = ARVRFixedTransform("hololens", "world", Vector3(0.975, -0.09, -0.27), Quaternion(0.0, 0.0, 1.0, 0.0), [[0, 0, 1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, -1]])
    mm = AR4LfDMiddleware(transform_manager, (.8, 0, -0.3), "start_ar", "/cairo_lfd/model_commands", "/cairo_lfd/lfd_representation", "/constraint_manager", "/constraint_edits", "/cairo_lfd/constraint_update")
    rospy.spin()


if __name__ == '__main__':
    main()
