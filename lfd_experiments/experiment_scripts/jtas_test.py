#! /usr/bin/env python
import argparse

import rospy
import intera_interface
from intera_interface import CHECK_VERSION

from robot_clients.trajectory_client import JointPositionTrajectory

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
    print(valid_limbs)
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
    traj = JointPositionTrajectory(limb, limb_interface.joint_names())
    print(limb_interface.joint_names())
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first
    limb_interface.move_to_neutral()
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    print(current_angles)
    traj.add_point(current_angles, 0.0)

    p1 = current_angles
    n_sec = 1.0
    traj.add_point(p1, n_sec)
    n_sec += 3.0
    traj.add_point([x * 0.6 for x in p1], n_sec)
    n_sec += 3.0
    traj.add_point([x * 0.3 for x in p1], n_sec)
    n_sec += 3.0
    traj.add_point([x * 0.6 for x in p1], n_sec)
    n_sec += 3.0
    traj.add_point([x * 0.3 for x in p1], n_sec)
    n_sec += 3.0
    traj.add_point([x * 0.6 for x in p1], n_sec)
    n_sec += 3.0
    traj.add_point([x * 1.0 for x in p1], n_sec)
    traj.start()
    traj.wait(n_sec)
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()