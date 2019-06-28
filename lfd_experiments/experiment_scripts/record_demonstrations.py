#!/usr/bin/env python2

import rospy
import argparse
import intera_interface
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import InteractionControlCommand
from geometry_msgs.msg import Pose
from intera_motion_interface import InteractionOptions, InteractionPublisher
from cairo_lfd.core.record import SawyerRecorder
from cairo_lfd.core.environment import Environment, import_configuration
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.modeling.analysis import ConstraintAnalyzer
from cairo_lfd.data.io import DataExporter
from cairo_lfd.data.processing import ProcessorPipeline, RelativeKinematicsProcessor, RelativePositionProcessor, InContactProcessor, SphereOfInfluenceProcessor


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

    required.add_argument(
        '-d', '--directory', dest='directory', required=True,
        help='the directory to save raw demonstration .json files'
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
    recorder = SawyerRecorder(args.record_rate, interaction_pub, interaction_options)
    rospy.on_shutdown(recorder.stop)
    rospy.on_shutdown(interaction_pub.send_position_mode_cmd)

    config_filepath = args.config
    configs = import_configuration(config_filepath)

    items = ItemFactory(configs).generate_items()
    constraints = ConstraintFactory(configs).generate_constraints()
    # We only have just the one robot...for now.......
    environment = Environment(items=items['items'], robot=items['robots'][0], constraints=constraints)

    exp = DataExporter()

    print("Recording. Press Ctrl-C to stop.")
    demos = recorder.record_demonstrations(environment, auto_zeroG=True)

    # Build processors and process demonstrations to generate derivative data e.g. relative position.
    rk_processor = RelativeKinematicsProcessor(environment.get_item_ids(), environment.get_robot_id())
    ic_processor = InContactProcessor(environment.get_item_ids(), environment.get_robot_id(), .06, .5)
    soi_processor = SphereOfInfluenceProcessor(environment.get_item_ids(), environment.get_robot_id())
    rp_processor = RelativePositionProcessor(environment.get_item_ids(), environment.get_robot_id())
    pipeline = ProcessorPipeline([rk_processor, ic_processor, soi_processor, rp_processor])
    pipeline.process(demos)

    # Analyze observations for constraints. 
    constraint_analyzer = ConstraintAnalyzer(environment)
    for demo in demos:
        constraint_analyzer.applied_constraint_evaluator(demo.observations)

    exp = DataExporter()
    for idx, demo in enumerate(demos):
        raw_data = [obs.data for obs in demo.observations]
        print("'/raw_demonstration{}.json': {} observations".format(idx, len(raw_data)))
        exp.export_to_json(args.directory + "/raw_demonstration{}.json".format(idx), raw_data)


if __name__ == '__main__':
    main()
