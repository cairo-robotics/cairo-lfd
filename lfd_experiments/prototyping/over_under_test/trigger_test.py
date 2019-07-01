#!/usr/bin/env python

import argparse
import rospy
import intera_interface
from lfd.triggers import SawyerCuffButtonTrigger


def main():
    rospy.init_node('trigger_test', anonymous=True)
    trigger = SawyerCuffButtonTrigger("right_button_triangle")
    # nav = intera_interface.Navigator()
    while not rospy.is_shutdown():
        print(trigger.check())
        # print(nav.get_button_state("right_button_circle"))


if __name__ == "__main__":
    main()