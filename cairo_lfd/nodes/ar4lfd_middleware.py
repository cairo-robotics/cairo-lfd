#!/usr/bin/env python

import rospy

from cairo_lfd.middleware.ar_middleware import AR4LfDMiddleware


def main():
    rospy.init_node("ar4lfd_middleware")
    mm = AR4LfDMiddleware("start_ar", "/cairo_lfd/model_commands", "/cairo_lfd/lfd_representation", "/constraint_manager")
    rospy.spin()


if __name__ == '__main__':
    main()
