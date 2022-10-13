#!/usr/bin/python3

import argparse
import rospy
import numpy as np
from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.data.conversion import convert_data_to_pose


def main():

    position = [0.6510909920817844, -0.32079366576278295, 0.15]
    # orientation = [0.999364381860628, -0.0321504866108988, 0.01184169368679194, -0.009846459751380035]
    orientation = [0.724142324757704, -0.011632070356660449, 0.687893404040879, 0.047804321047140386]

    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.55)
    moveit_interface.set_acceleration_scaling(.55)
    pose = convert_data_to_pose(position=position, orientation=orientation)
    moveit_interface.set_pose_target(pose)
    moveit_interface.execute(moveit_interface.plan())


if __name__ == '__main__':
    main()
