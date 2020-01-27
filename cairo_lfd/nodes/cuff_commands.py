#!/usr/bin/env python
import rospy

from cairo_lfd.core.record import CuffController


def main():
    rospy.init_node("cuff_controller")
    keyboard_ctl = CuffController()
    keyboard_ctl.run_loop()


if __name__ == '__main__':
    main()
