#!/usr/bin/python
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
        '-p', '--position', dest='position', type=list, default=[0, 1, 0],
        help='the x, y, z position of the end-effector'
    )

    parser.add_argument(
        '-o', '--orientation', dest='orientation', type=list, default=[0, 1, 0, 1],
        help='the x, y, z, w quaternion orientation of the end-effector'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    print(args.position)
    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.35)
    moveit_interface.set_acceleration_scaling(.25)

    pose = convert_data_to_pose(position=args.position, orientation=args.orientation)
    moveit_interface.execute(moveit_interface.plan())


if __name__ == '__main__':
    main()
