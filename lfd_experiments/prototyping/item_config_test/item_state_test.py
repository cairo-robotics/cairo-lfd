#!/usr/bin/env python2

import rospy
import argparse
import intera_interface
from intera_interface import CHECK_VERSION
from intera_motion_interface import InteractionOptions, InteractionPublisher
from cairo_lfd.core.record import SawyerRecorder
from cairo_lfd.core.environment import Environment, import_configuration
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.constrain.concept_constraints import ConstraintFactory
from cairo_lfd.modeling.analysis import ConstraintAnalyzer
from cairo_lfd.data.io import DataExporter



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
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()

    recorder = SawyerRecorder(args.record_rate)
    rospy.on_shutdown(recorder.stop)

    config_filepath = args.config
    configs = import_configuration(config_filepath)

    items = ItemFactory(configs).generate_items()
    constraints = ConstraintFactory(configs["constraints"]).generate_constraints()
    # We only have just the one robot...for now.......
    environment = Environment(items=items['items'], robot=items['robots'][0], constraints=constraints)

    exp = DataExporter()

    print("Recording. Press Ctrl-C to stop.")
    demos = recorder.record_demonstrations(environment)

    # Build processors and process demonstrations to generate derivative data e.g. relative position.
    rk_processor = RelativeKinematicsProcessor(environment.get_item_ids(), environment.get_robot_id())
    ic_processor = InContactProcessor(environment.get_item_ids(), environment.get_robot_id(), .06, .5)
    soi_processor = SphereOfInfluenceProcessor(environment.get_item_ids(), environment.get_robot_id())
    rp_processor = RelativePositionProcessor(environment.get_item_ids(), environment.get_robot_id())
    pipeline = ProcessorPipeline([rk_processor, ic_processor, soi_processor, rp_processor])
    pipeline.process(demonstrations)

    # Analyze for applied constraints. This will apply a set of constraints for all observations that for which the constraint set holds true. i.e. true until its not.
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
