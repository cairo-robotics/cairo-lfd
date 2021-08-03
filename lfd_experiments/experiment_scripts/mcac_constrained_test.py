#! /usr/bin/env python
import os
import json

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from intera_motion_interface.motion_trajectory import  MotionTrajectory
from intera_motion_interface.motion_waypoint import MotionWaypoint


def main():
    """
    """

    print("Initializing node... ")
    rospy.init_node("motion_control_example")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    limb_interface = intera_interface.Limb("right")
    abs_file_path = os.path.join(os.path.dirname(__file__), "constrained_traj.json")
    with open(abs_file_path, 'r') as f:
        data = json.load(f)
    start_position = dict(zip(limb_interface.joint_names(), data['trajectory'][0]['point']))
    limb_interface.move_to_joint_positions(start_position)

    print("Generating MotionTrajectory")
    traj = MotionTrajectory()
    rospy.on_shutdown(traj.stop_trajectory)

    # Command Current Joint Positions first
    for traj_pt in data['trajectory']:
        traj.append_waypoint(MotionWaypoint(traj_pt['point']))

    traj.send_trajectory()

    print("Exiting - Joint Motion Trajectory Test Complete")

if __name__ == "__main__":
    main()