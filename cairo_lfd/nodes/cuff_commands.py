#!/usr/bin/env python
import rospy

from cairo_lfd.controllers.cuff_controllers import CuffRecordingController


def main():
    rospy.init_node("cuff_controller")
    keyboard_ctl = CuffRecordingController()
    keyboard_ctl.run_loop()


if __name__ == '__main__':
    main()
