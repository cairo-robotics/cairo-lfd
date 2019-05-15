#!/usr/bin/env python

import rospy
import argparse
import intera_interface
from intera_interface import CHECK_VERSION
from lfd.environment import Environment, import_configuration
from lfd.items import ItemFactory
from lfd.constraints import ConstraintFactory
from lfd.data_io import DataImporter, DataExporter
from lfd.environment import Observation, Demonstration
from lfd.processing import ObjectRelativeDataProcessor, ObjectContactProcessor, SphereOfInfluenceProcessor
from lfd.segmentation import ManualSegmentation

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-c', '--config', dest='config', required=True,
        help='the file path of the demonstration '
    )

    required.add_argument(
        '-i', '--input', dest='input_directory', required=True,
        help='the input directory to pull in raw demonstration .json files'
    )

    required.add_argument(
        '-o', '--output', dest='output_directory', required=True,
        help='the output directory for saving relative distance labeled demonstration .json files'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    

    config_filepath = args.config
    configs = import_configuration(config_filepath)

    # Import the data
    importer = DataImporter()
    rospy.loginfo("Importing demonstration .json files...")
    raw_demonstrations = importer.load_json_files(args.input_directory + "/*.json")

    # Convert imported data into Demonstrations and Observations
    demonstrations = []
    for datum in raw_demonstrations["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(observations))
    args = parser.parse_args(rospy.myargv()[1:])

    # Process data
    ordp = ObjectRelativeDataProcessor([2, 3], 1)
    ocp = ObjectContactProcessor([2, 3], 1, .06, .5)
    soip = SphereOfInfluenceProcessor([2, 3], 1, .25)

    for idx, demo in enumerate(demonstrations):
        ordp.generate_relative_data(demo.observations)
        ocp.generate_object_contact_data(demo.observations)
        soip.generate_SOI_data(demo.observations)
    print("\nDone processing.")

    # Segment data
    segmenter = ManualSegmentation()
    for idx, demo in enumerate(demonstrations):
        segmenter.segment(demo.observations)
    print("\nDone segment.")

    # Export data
    rospy.loginfo("Exporting demonstrations.")
    exp = DataExporter()
    for idx, demo in enumerate(demonstrations):
        labeled_data = [obs.data for obs in demo.observations]
        exp.export_to_json(args.output_directory + "/labeled_demonstration{}.json".format(idx), labeled_data)
    print("\nJust plain done.")

if __name__ == '__main__':
    main()