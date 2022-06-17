#!/usr/bin/python3

import argparse
import rospy
import numpy as np
from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.data.conversion import convert_data_to_pose


def main():

    position = [ 0.70497801598107912, 0.032547560252505,  .1766266588857409]
    # orientation = [0.999364381860628, -0.0321504866108988, 0.01184169368679194, -0.009846459751380035]
    orientation = [-0.5025809059472035, -0.455533381558173, 0.5438250996747817, -0.4941214751730367]

    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.55)
    moveit_interface.set_acceleration_scaling(.55)
    pose = convert_data_to_pose(position=position, orientation=orientation)
    moveit_interface.set_pose_target(pose)
    moveit_interface.execute(moveit_interface.plan())


if __name__ == '__main__':
    main()
