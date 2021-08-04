#! /usr/bin/env python

import os
import sys
import argparse
import json

import rospy
import intera_interface
from intera_interface import CHECK_VERSION

from robot_clients.trajectory_client import JTASWithIDClient

def main():
    """SDK Joint Trajectory Example: Simple Action Client

    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.

    Make sure to start the joint_trajectory_action_server.py
    first. Then run this example on a specified limb to
    command a short series of trajectory points for the arm
    to follow.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
          "Exiting."), "ERROR")
        return

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=valid_limbs, default=valid_limbs[0],
        help='send joint trajectory to which limb'
    )

    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb

    print("Initializing node... ")
    rospy.init_node("sdk_joint_trajectory_client_{0}".format(limb))
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    limb_interface = intera_interface.Limb(limb)
    limb_interface.set_joint_position_speed(speed=.3)
    traj = JTASWithIDClient(limb, limb_interface.joint_names())
    print(limb_interface.joint_names())
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first
    abs_file_path = os.path.join(os.path.dirname(__file__), "traj_w_id.json")
    with open(abs_file_path, 'r') as f:
        data = json.load(f)
    start_position = dict(zip(limb_interface.joint_names(), data['trajectory'][0]['position']))
    limb_interface.move_to_joint_positions(start_position)
    wait_duration = data['trajectory'][-1]['time']

    for traj_pt in data['trajectory']:
        traj.add_point(traj_pt['position'], traj_pt['velocity'], traj_pt['acceleration'], traj_pt['time']/5)

    traj.start()
    traj.wait(wait_duration)

    print("Exiting - Joint Trajectory Action w/ ID Test Complete")

if __name__ == "__main__":
    main()