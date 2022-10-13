#!/usr/bin/python3
import argparse
import rospy
from robot_interface.moveit_interface import SawyerMoveitInterface
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

    joints = [-0.225095703125, 1.035513671875, -1.508908203125, 0.9075078125, -2.3465009765625, 1.3790859375, 2.747173828125]
    # joints = [-6.451021006359525e-05, 0.024965327824957306, -1.5229091964871966, 1.569756788173077, -0.004876280173287562, -1.5704467591066407, 0.00018739142623847016]
    # joints = args.joints
    moveit_interface = SawyerMoveitInterface(tip_names=["right_hand"])
    results = moveit_interface.forward_kinematics(joints)
    position = [results.position.x, results.position.y, results.position.z]
    orientation = [results.orientation.x, results.orientation.y, results.orientation.z, results.orientation.w]
    print(position, quat2rpy(orientation), orientation)


if __name__ == '__main__':
    main()
