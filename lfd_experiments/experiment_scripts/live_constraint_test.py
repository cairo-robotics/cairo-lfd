#!/usr/bin/env python2

import rospy
import argparse
import intera_interface
import sys
import select
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import InteractionControlCommand
from geometry_msgs.msg import Pose
from intera_motion_interface import InteractionOptions, InteractionPublisher
from lfd.record import SawyerRecorder
from lfd.environment import Environment, Observation, import_configuration
from lfd.items import ItemFactory
from lfd.constraints import ConstraintFactory
from lfd.analysis import ConstraintAnalyzer
from lfd.data_io import DataExporter


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
    rospy.init_node("sdk_joint_recorder")
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
    args.record_rate, interaction_pub, interaction_options
    rospy.on_shutdown(interaction_pub.send_position_mode_cmd)
    interaction_pub.external_rate_send_command(interaction_options)
    config_filepath = args.config
    configs = import_configuration(config_filepath)

    items = ItemFactory(configs).generate_items()
    constraints = ConstraintFactory(configs).generate_constraints()
    constraint_ids = [constraint.id for constraint in constraints]
    print("Constraint IDs: {}".format(constraint_ids))
    # We only have just the one robot...for now.......
    environment = Environment(items=items['items'], robot=items['robots'][0], constraints=constraints)

    constraint_analyzer = ConstraintAnalyzer(environment)

    user_input = ""
    while environment.robot._navigator.get_button_state("right_button_back") != 2 or user_input == "q":
        stdin, stdout, stderr = select.select([sys.stdin], [], [], .0001)
        for s in stdin:
            if s == sys.stdin:
                user_input = sys.stdin.readline().strip()
        data = {
            "robot": environment.get_robot_state(),
            "items": environment.get_item_state(),
            "triggered_constraints": environment.check_constraint_triggers()
        }
        observation = Observation(data)
        print(constraint_analyzer.evaluate(constraint_ids, observation))
        rospy.sleep(1)
        if rospy.is_shutdown():
            return 1

if __name__ == '__main__':
    main()
