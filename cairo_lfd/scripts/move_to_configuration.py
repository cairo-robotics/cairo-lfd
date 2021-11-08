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

    configuration = [-1.3873709693792482, 0.6889105515036634, -1.3040140417968091, -0.6337524498504346, -0.412876815322056, -0.6817950614592864, -3.3164909766471973]
    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.55)
    moveit_interface.set_acceleration_scaling(.55)
    moveit_interface.set_joint_target(configuration)
    moveit_interface.execute(moveit_interface.plan())


if __name__ == '__main__':
    main()
