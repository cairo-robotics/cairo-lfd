#!/usr/bin/python
import argparse
import rospy
from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.data.conversion import convert_data_to_pose
from scipy.spatial.transform import Rotation as R


def quat2rpy(xyzw, degrees=False):
    """
    Converts a quaternion in xyzw form to euler angle rotation xyz/rpy extrinsic form.

    Args:
        xyzw (array-like): xyzw quaternion vector.
        degrees (bool, optional): False for radians, True for degrees. Defaults to False.

    Returns:
        ndarray: Returns the rpy angles of the quaternion.
    """
    r = R.from_quat([xyzw[0], xyzw[1], xyzw[2], xyzw[3]])
    return r.as_euler("xyz", degrees=degrees)

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-j', '--joints', dest='joints', nargs=3, type=float,
        help='7 dof joints'
    )

    args = parser.parse_args(rospy.myargv()[1:])
    joints = [-1.473982421875, 0.826048828125, -1.175953125, -0.94485546875, -0.790203125, -0.5344697265625, -0.11094921875]
    # joints = [-6.451021006359525e-05, 0.024965327824957306, -1.5229091964871966, 1.569756788173077, -0.004876280173287562, -1.5704467591066407, 0.00018739142623847016]
    # joints = args.joints
    moveit_interface = SawyerMoveitInterface()
    results = moveit_interface.forward_kinematics(joints)
    position = [results.position.x, results.position.y, results.position.z]
    orientation = [results.orientation.x, results.orientation.y, results.orientation.z, results.orientation.w]
    print(position, quat2rpy(orientation))


if __name__ == '__main__':
    main()
