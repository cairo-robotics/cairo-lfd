#!/usr/bin/env python2

import rospy
import argparse
import intera_interface
from intera_interface import CHECK_VERSION
from lfd.record import SawyerRecorder
from lfd.environment import Environment, import_configuration
from lfd.items import ItemFactory
from lfd.constraints import ConstraintFactory
from lfd.analyzer import ConstraintAnalyzer
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
