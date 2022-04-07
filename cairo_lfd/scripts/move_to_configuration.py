#!/usr/bin/python3

import argparse
import rospy
import numpy as np
from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.data.conversion import convert_data_to_pose


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    configuration = [ 0.0, 0.0, -1.5708, 1.5708, 0.0, -1.5708, 0.0 ]
    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.55)
    moveit_interface.set_acceleration_scaling(.55)
    moveit_interface.set_joint_target(configuration)
    moveit_interface.execute(moveit_interface.plan())


if __name__ == '__main__':
    main()
