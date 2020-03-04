#!/usr/bin/env python
import rospy

from cairo_lfd.controllers.keyboard_controllers import ModelKeyboardController


def main():
    rospy.init_node("model_controller")
    keyboard_ctl = ModelKeyboardController()
    keyboard_ctl.run_loop()


if __name__ == '__main__':
    main()
