#!/usr/bin/python3

import argparse
import rospy
import numpy as np
from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.data.conversion import convert_data_to_pose


def main():
    position = [0.796822468835666, -0.5726076696256587, -0.06436333069221895]
    # orientation = [0.999364381860628, -0.0321504866108988, 0.01184169368679194, -0.009846459751380035]
    orientation = [0.6955931963895514, -0.00599257612720292, 0.7183945532979211, -0.004843548266260681]

    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.55)
    moveit_interface.set_acceleration_scaling(.55)
    pose = convert_data_to_pose(position=position, orientation=orientation)
    moveit_interface.set_pose_target(pose)
    moveit_interface.execute(moveit_interface.plan())


if __name__ == '__main__':
    main()
