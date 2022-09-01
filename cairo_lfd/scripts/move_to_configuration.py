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

    configuration = [-1.3189136841166726, 0.485822174871958, -1.2973158681981019, -0.8143342002873153, -0.3846951412628785, -0.4319590127219599, 3.090875503125326]
    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.55)
    moveit_interface.set_acceleration_scaling(.55)
    moveit_interface.set_joint_target(configuration)
    moveit_interface.execute(moveit_interface.plan())


if __name__ == '__main__':
    main()
