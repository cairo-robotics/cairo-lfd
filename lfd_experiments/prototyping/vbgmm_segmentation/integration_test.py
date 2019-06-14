#!/usr/bin/python

import os

from lfd.environment import Demonstration, Observation
from lfd.data_io import DataImporter
from lfd.data_conversion import vectorize_demonstration
from lfd.segmentation import DemonstrationSegmentGenerator, VariationalGMMSegmenter
# from modeling import GaussianMixtureComponent3D


def get_sequence(segment):
    pass

def main():
    # Build model
    dir_path = os.path.join(os.path.dirname(
        os.path.realpath(__file__)), 'place_atop_demos')
    training_data = DataImporter().load_json_files(os.path.join(dir_path, '*.json'))

    # Convert imported data into Demonstrations and Observations
    demonstrations = []
    for datum in training_data["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(observations))

    gmm_segmenter = VariationalGMMSegmenter(
        demonstrations, vectorize_demonstration, n_components=5)
    model = gmm_segmenter.model
    # Deploy model to segment each demonstration
    demo_segmenter = DemonstrationSegmentGenerator(gmm_segmenter)

    segments = demo_segmenter.segment_demonstrations(demonstrations)
    print type(segments)
    for demo_id, segment in segments.iteritems():
        print segment['order']


if __name__ == '__main__':
    main()
