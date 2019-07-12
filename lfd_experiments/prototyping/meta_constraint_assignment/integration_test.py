#!/usr/bin/python

import os
from functools import partial

import numpy as np

from cairo_lfd.core.environment import Demonstration, Observation
from cairo_lfd.data.io import DataImporter
from cairo_lfd.data.vectorization import vectorize_demonstration, vectorize_robot_position
from cairo_lfd.constraints.segmentation import BayesianGMMSegmentModel
# from modeling import GaussianMixtureComponent3D


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

    position_vectors = np.array(map(partial(vectorize_demonstration, vectorizors=[vectorize_robot_position]), demonstrations))
    X = np.vstack(position_vectors)

    height_segment_model = BayesianGMMSegmentModel(X, n_components=100)

    print(height_segment_model._get_active_components())
    print(height_segment_model.predict(position_vectors[0]))


if __name__ == '__main__':
    main()
