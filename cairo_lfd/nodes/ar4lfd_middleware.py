#!/usr/bin/env python3

import rospy

from cairo_lfd.middleware.ar_middleware import AR4LfDMiddleware


def main():
    rospy.init_node("ar4lfd_middleware")
    mm = AR4LfDMiddleware("start_ar", "/cairo_lfd/model_commands", "/cairo_lfd/lfd_representation", "/constraint_manager", "/constraint_edits", "/cairo_lfd/constraint_update")
    rospy.spin()


if __name__ == '__main__':
    main()
