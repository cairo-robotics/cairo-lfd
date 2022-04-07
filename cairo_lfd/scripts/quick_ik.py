#!/usr/bin/python3

import argparse
import rospy
from robot_interface import SawyerMoveitInterface
from cairo_lfd.data.conversion import convert_data_to_pose


def main():
    position = [0.39511943327816434, 0.469853990842666, 0.8093717344145103]
    orientation = [0.999364381860628, -0.0321504866108988, 0.01184169368679194, -0.009846459751380035]
    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    pose = convert_data_to_pose(position=position, orientation=orientation)

    print(moveit_interface.get_IK_pose(pose))


if __name__ == '__main__':
    main()
