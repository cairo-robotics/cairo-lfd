#!/usr/bin/env python

import rospy
import argparse
import intera_interface
from intera_interface import CHECK_VERSION
from cairo_lfd.core.environment import Environment, Observation, Demonstration
from cairo_lfd.data.io import load_json_files, export_to_json, load_lfd_configuration
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.modeling.graphing import IntermediateTrajectories

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    # required.add_argument(
    #     '-c', '--config', dest='config', required=True,
    #     help='the file path of the demonstration '
    # )

    required.add_argument(
        '-i', '--input', dest='input_directory', required=True,
        help='the input directory to pull in raw demonstration .json files'
    )

    # required.add_argument(
    #     '-o', '--output', dest='output_directory', required=True,
    #     help='the output directory for saving relative distance labeled demonstration .json files'
    # )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("intermediate_trajectories")
    # print("Getting robot state... ")
    # rs = intera_interface.RobotEnable(CHECK_VERSION)
    # print("Enabling robot... ")
    # rs.enable()

    # config_filepath = args.config
    # configs = load_lfd_configuration(config_filepath)

    # items = ItemFactory(configs).generate_items()
    # constraints = ConstraintFactory(configs["constraints"]).generate_constraints()
    # # We only have just the one robot...for now.......
    # environment = Environment(items=items['items'], robot=items['robots'][0], constraints=constraints)

    # Import the data
    rospy.loginfo("Importing demonstration .json files...")
    raw_demonstrations = load_json_files(args.input_directory + "/*.json")

    # Convert imported data into Demonstrations and Observations
    demonstrations = []
    for datum in raw_demonstrations["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(None, labeled_observations=observations))
    args = parser.parse_args(rospy.myargv()[1:])
    
    inter_trajectories = IntermediateTrajectories().get_trajectories(demonstrations)
    print(inter_trajectories)
    print(len(inter_trajectories[13][0]))
    print(len(inter_trajectories[13][1]))
    print(len(inter_trajectories[13][2]))

    # for idx, demo in enumerate(demonstrations):
    #     labeled_data = [obs.data for obs in demo.observations]
    #     export_to_json(args.output_directory + "/labeled_demonstration{}.json".format(idx), labeled_data)
    # print("\nDone.")


if __name__ == '__main__':
    main()