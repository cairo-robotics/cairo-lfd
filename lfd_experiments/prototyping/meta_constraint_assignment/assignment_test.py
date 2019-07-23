#!/usr/bin/python

import os
from functools import partial

import numpy as np

import rospy

from cairo_lfd.core.environment import Demonstration, Observation
from cairo_lfd.data.io import DataImporter
from cairo_lfd.data.vectorization import vectorize_demonstration, vectorize_robot_position, vectorize_robot_orientation
from cairo_lfd.constraints.segmentation import BayesianGMMSegmentModel
from cairo_lfd.constraints.meta_constraint import MetaConstraintBuilder, HeightMetaConstraint
from cairo_lfd.modeling.graphing import KeyframeGraph
from cairo_lfd.modeling.graphing import ObservationClusterer, KeyframeGraph
from cairo_lfd.modeling.models import KDEModel


def main():
    # Build model
    dir_path = os.path.join(os.path.dirname(
        os.path.realpath(__file__)), 'labeled')
    training_data = DataImporter().load_json_files(os.path.join(dir_path, '*.json'))

    # Convert imported data into Demonstrations and Observations
    demonstrations = []
    for datum in training_data["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(observations))

    position_vectorizor = partial(vectorize_demonstration, vectorizors=[vectorize_robot_position])
    position_vectors = np.array(map(position_vectorizor, demonstrations))
    X = np.vstack(position_vectors)

    height_segment_model = BayesianGMMSegmentModel(X, n_components=100)

    graph = KeyframeGraph()
    cluster_generator = ObservationClusterer()

    """
    Generate clusters using labeled observations, build the models, graphs, and atributes for each
    cluster in the KeyFrameGraph
    """
    clusters = cluster_generator.generate_clusters(demonstrations)
    for cluster_id in clusters.keys():
        graph.add_node(cluster_id)
        graph.nodes[cluster_id]["observations"] = clusters[cluster_id]["observations"]
        graph.nodes[cluster_id]["keyframe_type"] = clusters[cluster_id]["keyframe_type"]
        graph.nodes[cluster_id]["applied_constraints"] = clusters[cluster_id]["applied_constraints"]
        graph.nodes[cluster_id]["meta_constraints"] = {"height": []}
        graph.nodes[cluster_id]["model"] = KDEModel(kernel='gaussian', bandwidth=.25)
    graph.add_path(graph.nodes())
    graph.fit_models(vectorize_robot_orientation)
    graph._identify_primal_observations(vectorize_robot_orientation)
    rospy.loginfo(graph.get_keyframe_sequence())

    heigh_constraint_builder = MetaConstraintBuilder(height_segment_model, HeightMetaConstraint)
    for node in graph.get_keyframe_sequence():
        vectors = []
        for observation in graph.nodes[node]["observations"]:
            vectors.append(vectorize_robot_position(observation))
        graph.nodes[node]["meta_constraints"]["height"] = heigh_constraint_builder.generate_meta_constraints(vectors)
        for constraint in graph.nodes[node]["meta_constraints"]["height"]:
            print(constraint)

    builder = MetaConstraintBuilder(height_segment_model, HeightMetaConstraint)
    builder.generate_meta_constraint()


if __name__ == '__main__':
    main()
