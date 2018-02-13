#!/usr/bin/env python2.7
import rospy
import argparse
import os
from lfd_processor.environment import Demonstration, Observation
from lfd_processor.alignment import DemonstrationAligner, vectorize_demonstration
from lfd_processor.analyzer import DemonstrationKeyframeLabeler
from lfd_processor.data_io import DataExporter
from lfd_processor.data_io import DataImporter


if __name__ == "__main__":

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-d', '--directory', dest='directory', required=True,
        help='the directory to output aligned demonstration .json files.'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    importer = DataImporter()
    direc = os.path.dirname(__file__)
    filename = os.path.join(direc, "../toy_data/constrained_trajectories/*.json")
    trajectories = importer.load_json_files(filename)

    # Convert trajectory data into Demonstrations and Observations
    demonstrations = []
    for datum in trajectories["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(observations))

    aligner = DemonstrationAligner(demonstrations, vectorize_demonstration)
    aligned_demos, constraint_transitions = aligner.align()

    keyframe_grouper = DemonstrationKeyframeLabeler(aligned_demos, constraint_transitions)
    labeled_demonstrations = keyframe_grouper.label_demonstrations(20, 10)

    exp = DataExporter()
    for idx, demo in enumerate(labeled_demonstrations):
        raw_data = [obs.data for obs in demo.labeled_observations]
        exp.export_to_json(args.directory + "/labeled_demonstration{}.json".format(idx), raw_data)

    print "Please see the labeled_demonstration*.json files located at {} to see your results.".format(args.directory)
