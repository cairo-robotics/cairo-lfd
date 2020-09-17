#!/usr/bin/env python
import rospy

from cairo_lfd.controllers.cuff_controllers import RecordingCuffController


def main():
    rospy.init_node("cuff_controller")
    keyboard_ctl = RecordingCuffController()
    keyboard_ctl.run_loop()


if __name__ == '__main__':
    main()
