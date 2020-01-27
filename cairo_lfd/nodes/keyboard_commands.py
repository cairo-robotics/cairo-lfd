#!/usr/bin/env python
import rospy

from cairo_lfd.core.record import KeyboardController


def main():
    rospy.init_node("keyboard_controller")
    keyboard_ctl = KeyboardController()
    keyboard_ctl.run_loop()


if __name__ == '__main__':
    main()
