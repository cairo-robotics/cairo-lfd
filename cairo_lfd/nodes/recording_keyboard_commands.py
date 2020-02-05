#!/usr/bin/env python
import rospy

from cairo_lfd.controllers.keyboard_controllers import RecordingKeyboardController


def main():
    rospy.init_node("keyboard_controller")
    keyboard_ctl = RecordingKeyboardController()
    keyboard_ctl.run_loop()


if __name__ == '__main__':
    main()
