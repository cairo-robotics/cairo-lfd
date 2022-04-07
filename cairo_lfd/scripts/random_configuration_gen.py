#!/usr/bin/python3

import argparse
import rospy
from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.data.conversion import convert_data_to_pose


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-p', '--position', dest='position', nargs=3, type=float,
        help='the x, y, z position of the end-effector'
    )

    parser.add_argument(
        '-o', '--orientation', dest='orientation', nargs=4, type=float,
        help='the x, y, z, w quaternion orientation of the end-effector'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    print(args.position)
    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    pose = convert_data_to_pose(position=args.position, orientation=args.orientation)
    
    print(moveit_interface.get_IK_pose(pose))


if __name__ == '__main__':
    main()
