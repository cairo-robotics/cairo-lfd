#!/usr/bin/env python

import argparse
import rospy

from robot_interface.moveit_interface import SawyerMoveitInterface


def main():
    joints = [0.49774609375, 1.0437587890625, -2.2413759765625, 1.0047255859375, 2.8415927734375, 1.2034716796875, -2.499330078125]
    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    robot_state = moveit_interface.create_robot_state(joints)
    print(robot_state)
    print(moveit_interface.check_point_validity(robot_state))
    print(moveit_interface.get_FK_pose(joints))

if __name__ == "__main__":
    main()