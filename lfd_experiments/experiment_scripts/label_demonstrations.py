#!/usr/bin/env python

from functools import partial

import rospy
import argparse

from cairo_lfd.data.io import DataImporter, DataExporter
from cairo_lfd.data.vectorization import vectorize_demonstration, vectorize_robot_position
from cairo_lfd.data.alignment import DemonstrationAligner
from cairo_lfd.data.labeling import ConstraintKeyframeLabeler
from cairo_lfd.core.environment import Observation, Demonstration


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-i', '--input_directory', dest='input_directory', required=True,
        help='the directory from which to input raw demonstration .json files'
    )
    parser.add_argument(
        '-o', '--output_directory', dest='output_directory', required=True,
        help='the directory to save labeled demonstration .json files'
    )

    parser.add_argument(
        '-d', '--divisor', type=int, default=20, metavar='DIVISOR',
        help='the total number of observations per n number of observations (divisor) determining number of keyframes'
    )

    parser.add_argument(
        '-w', '--window', type=int, default=10, metavar='WINDOW',
        help='window size of number of observations per keyframe to capture'
    )
    args = parser.parse_args(rospy.myargv()[1:])

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

    # For more details about alignment, see the alignment_example.
    rospy.loginfo("Aligning demonstrations...this can take some time.")
    aligner = DemonstrationAligner(demonstrations, partial(vectorize_demonstration, vectorizors=[vectorize_robot_position]))
    aligned_demos, constraint_transitions = aligner.align()

    # Create DemosntrationkeyframeLabeler passing in the aligned demonstrations and the constraint transition
    # ordering, both of which are returned from the DemonstratinAligner object.
    keyframe_labeler = ConstraintKeyframeLabeler(aligned_demos, constraint_transitions)
    """Call label_demontrations. The first parameter is the average group length divisor which determines the number of keyframes for that group. So if the first grouping of data before the first constraint transition has an average length of 100, then that first grouping will generate 5 keyframes (100/20 = 5). 

    The second parameter is the window size i.e. how big each keyframe size should be (+/- one depending on if odd number of elements in the grouping list per demonstration) 
    """
    rospy.loginfo("Labeling demonstrations.")
    labeled_demonstrations = keyframe_labeler.label_demonstrations(args.divisor, args.window)

    # Export the dictionary data representation of the observations of the labeled demos.
    # Notice that the demonstrations now have a populated instance member 'labeled_observations'.
    rospy.loginfo("Exporting demonstrations.")
    exp = DataExporter()
    for idx, demo in enumerate(labeled_demonstrations):
        labeled_data = [obs.data for obs in demo.labeled_observations]
        exp.export_to_json(args.output_directory + "/labeled_demonstration{}.json".format(idx), labeled_data)
    print("\nDone.")


if __name__ == '__main__':
    main()