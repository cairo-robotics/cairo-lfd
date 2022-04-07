#!/usr/bin/env python3

import rospy
import argparse
import intera_interface
import sys
import select
import transforms3d
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import InteractionControlCommand
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8MultiArray, String
from intera_motion_interface import InteractionOptions, InteractionPublisher
from cairo_lfd.core.environment import Environment, Observation
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.constraints.triggers import TriggerFactory
from cairo_lfd.modeling.analysis import check_constraint_validity
from cairo_lfd.data.io import load_lfd_configuration

import os
import argparse
import sys
import select

import transforms3d
import rospy
import intera_interface
from intera_interface import CHECK_VERSION

from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.core.record import SawyerDemonstrationRecorder, SawyerDemonstrationLabeler
from cairo_lfd.core.environment import Observation, Demonstration
from cairo_lfd.data.io import load_json_files, load_lfd_configuration
from cairo_lfd.data.vectorization import vectorize_demonstration, get_observation_joint_vector
from cairo_lfd.data.alignment import DemonstrationAlignment
from cairo_lfd.data.processing import DataProcessingPipeline, RelativeKinematicsProcessor, RelativePositionProcessor, InContactProcessor, SphereOfInfluenceProcessor, WithinPerimeterProcessor
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.core.robots import RobotFactory
from cairo_lfd.core.lfd import CC_LFD
from cairo_lfd.controllers.study_controllers import CCLfDController


def main():
    """
    Demonstration Recorder

    Record a series of demonstrations.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-c', '--config', dest='config', required=True,
        help='the file path of the demonstration '
    )

    parser.add_argument(
        '-r', '--record_rate', type=int, default=45, metavar='RECORDRATE',
        help='rate at which to record (default: 45)'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("live_constraint")
    print("Getting robot state... ")
    robot_state = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    robot_state.enable()

    interaction_pub = InteractionPublisher()
    interaction_options = InteractionOptions()
    interaction_options.set_max_impedance([False])
    interaction_options.set_rotations_for_constrained_zeroG(True)
    interaction_frame = Pose()
    interaction_frame.position.x = 0
    interaction_frame.position.y = 0
    interaction_frame.position.z = 0
    interaction_frame.orientation.x = 0
    interaction_frame.orientation.y = 0
    interaction_frame.orientation.z = 0
    interaction_frame.orientation.w = 1
    interaction_options.set_K_impedance([0, 0, 0, 0, 0, 0])
    interaction_options.set_K_nullspace([5, 5, 5, 5, 5, 5, 5])
    interaction_options.set_interaction_frame(interaction_frame)
    rospy.loginfo(interaction_options.to_msg())

    rospy.on_shutdown(interaction_pub.send_position_mode_cmd)
    interaction_pub.external_rate_send_command(interaction_options)
    config_filepath = args.config
    configs = load_lfd_configuration(config_filepath)

    robots = RobotFactory(configs['robots']).generate_robots()
    items = ItemFactory(configs['items']).generate_items()
    triggers = TriggerFactory(configs['triggers']).generate_triggers()
    constraints = ConstraintFactory(configs["constraints"]).generate_constraints()
    constraint_ids = [constraint.id for constraint in constraints]
    print("Constraint IDs: {}".format(constraint_ids))
    # We only have just the one robot...for now.......
    environment = Environment(
            items=items, robot=robots[0], constraints=constraints, triggers=triggers)

    user_input = ""
    while environment.robot._navigator.get_button_state("right_button_back") != 2 or user_input == "q":
        stdin, _, _ = select.select([sys.stdin], [], [], .0001)
        for s in stdin:
            if s == sys.stdin:
                user_input = sys.stdin.readline().strip()
        data = {
            "robot": environment.get_robot_state(),
            "items": environment.get_item_state(),
            "triggered_constraints": environment.check_constraint_triggers()
        }
        observation = Observation(data)
        euler = transforms3d.euler.quat2euler(data["robot"]["orientation"], axes='sxyz')
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        print("Position: " + str(data["robot"]["position"]))
        print("Orientation: " + str(data["robot"]["orientation"]))
        print("RPY: " + "{}, {}, {}".format(roll, pitch, yaw))
        print("Configuration: " + str(data["robot"]["joint_angle"]))
        print(check_constraint_validity(environment, constraints, observation))
        print(data["triggered_constraints"])
        valid_constraints = check_constraint_validity(environment, constraints, observation)[1]
        pub = rospy.Publisher('cairo_lfd/valid_constraints', Int8MultiArray, queue_size=10)
        msg = Int8MultiArray(data=valid_constraints)
        pub.publish(msg)
        rospy.sleep(1)
        if rospy.is_shutdown():
            return 1

if __name__ == '__main__':
    main()
