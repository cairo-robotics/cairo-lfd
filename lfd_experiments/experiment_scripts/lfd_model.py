#!/usr/bin/python

import argparse
import colored_traceback
import rospy

from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.core.lfd import LFD
from cairo_lfd.core.environment import Observation, Demonstration
from cairo_lfd.data.io import DataImporter, import_configuration

colored_traceback.add_hook()


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-c', '--config', dest='config', required=True,
        help='the file path of configuration config.json file '
    )

    required.add_argument(
        '-d', '--directory', dest='directory', required=True,
        help='the directory from which to input labeled demonstration .json files'
    )

    parser.add_argument(
        '-b', '--bandwidth', type=float, default=.025, metavar='BANDWIDTH',
        help='gaussian kernel density bandwidth'
    )

    parser.add_argument(
        '-t', '--threshold', type=int, metavar='THRESHOLD',
        help='log-liklihood threshold value'
    )

    parser.add_argument(
        '-n', '--number_of_samples', type=int, default=50, metavar='NUMBEROFSAMPLES',
        help='the number of samples to validate for each keyframe'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("graph_traverse")
    # Import the data
    importer = DataImporter()
    config_filepath = args.config
    configs = import_configuration(config_filepath)
    labeled_demonstrations = importer.load_json_files(args.directory + "/*.json")

    # Convert imported data into Demonstrations and Observations
    demonstrations = []
    for datum in labeled_demonstrations["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(observations))

    if len(demonstrations) == 0:
        rospy.logwarn("No demonstration data to model!!")
        return 0

        """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.35)
    moveit_interface.set_acceleration_scaling(.25)
    moveit_interface.set_planner(str(configs["settings"]["planner"]))

    lfd = LFD(configs, moveit_interface)
    lfd.build_environment()
    lfd.build_keyframe_graph(demonstrations, args.bandwidth)
    if args.threshold is not None:
        rospy.loginfo("Using user provided culling threshold of {}".format(args.threshold))
        lfd.sample_keyframes(args.number_of_samples, automate_threshold=False, culling_threshold=args.threshold)
    else:
        lfd.sample_keyframes(args.number_of_samples, automate_threshold=True)
    lfd.perform_skill()

if __name__ == '__main__':
    main()
