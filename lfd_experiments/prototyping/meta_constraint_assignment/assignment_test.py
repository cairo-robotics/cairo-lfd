#!/usr/bin/python

import os
from functools import partial

import numpy as np

from cairo_lfd.core.environment import Demonstration, Observation
from cairo_lfd.data.io import DataImporter
from cairo_lfd.data.vectorization import vectorize_demonstration, vectorize_robot_position
from cairo_lfd.constraints.segmentation import BayesianGMMSegmentModel
from cairo_lfd.constraints.meta_constraint import MetaConstraintAssignment, MetaConstraintFactory, HeightMetaConstraint

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

    position_vectors = np.array(map(partial(vectorize_demonstration, vectorizors=[vectorize_robot_position]), demonstrations))
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
        graph.nodes[cluster_id]["model"] = KDEModel(kernel='gaussian', bandwidth=args.bandwidth)
    graph.add_path(graph.nodes())
    graph.fit_models(get_observation_joint_vector)
    graph._identify_primal_observations(get_observation_joint_vector)
    rospy.loginfo(graph.get_keyframe_sequence())
    for node in graph.get_keyframe_sequence():
        print("KEYFRAME: {}".format(node))
        print(graph.nodes[node]["keyframe_type"])
        print(graph.nodes[node]["applied_constraints"])
        print 



if __name__ == '__main__':
    main()
