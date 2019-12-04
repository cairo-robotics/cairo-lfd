#!/usr/bin/python

import os
import argparse
from functools import partial
import random
import colored_traceback

import numpy as np
import rospy

from robot_interface.moveit_interface import SawyerMoveitInterface

from cairo_lfd.core.environment import Demonstration, Observation, Environment, import_configuration
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.data.io import DataImporter
from cairo_lfd.data.vectorization import vectorize_demonstration, vectorize_robot_position, vectorize_robot_orientation
from cairo_lfd.data.conversion import SawyerSampleConverter
from cairo_lfd.data.vectorization import get_observation_joint_vector
from cairo_lfd.constraints.segmentation import BayesianGMMSegmentModel
from cairo_lfd.constraints.heuristics import HeightHeuristicModel
from cairo_lfd.constraints.metaconstraints import HeightMetaconstraint
from cairo_lfd.constraints.metaconstraint_assignment import MetaconstraintAssigner, HeightMetaconstraintBuilder
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.modeling.graphing import ObservationClusterer, KeyframeGraph
from cairo_lfd.modeling.models import KDEModel
from cairo_lfd.modeling.sampling import KeyframeSampler
from cairo_lfd.modeling.analysis import KeyframeGraphAnalyzer, ConstraintAnalyzer


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
        '-t', '--threshold', type=int, default=-1200, metavar='THRESHOLD',
        help='log-liklihood threshold value'
    )

    parser.add_argument(
        '-n', '--number_of_samples', type=int, default=50, metavar='NUMBEROFSAMPLES',
        help='the number of samples to validate for each keyframe'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    # Import the data
    importer = DataImporter()
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

    rospy.init_node("graph_traverse")

    """ Create the Cairo LfD environment """
    config_filepath = args.config
    configs = import_configuration(config_filepath)
    items = ItemFactory(configs).generate_items()
    constraints = ConstraintFactory(configs).generate_constraints()
    # We only have just the one robot...for now.......
    environment = Environment(items=items['items'], robot=items['robots'][0], constraints=constraints, triggers=None)

    """ Create the moveit_interface """
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.35)
    moveit_interface.set_acceleration_scaling(.25)

    """ Create KeyframeGraph object. """
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
        graph.nodes[cluster_id]["meta_constraints"] = {}
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

    # Create height segmentation and heuristic model
    position_vectorizor = partial(vectorize_demonstration, vectorizors=[vectorize_robot_position])
    position_vectors = np.array(map(position_vectorizor, demonstrations))
    # stack all observation vectors
    X = np.vstack(position_vectors)
    height_segment_model = BayesianGMMSegmentModel(X, n_components=10)
    height_heuristic = HeightHeuristicModel(height_segment_model)
    height_heuristic.fit()
    height_static_parameters = {
        "item_id": 1,
        "reference_height": 0,
        "direction": "positive"
    }
    height_metaconstraint_builder = HeightMetaconstraintBuilder(height_heuristic, height_static_parameters)
    metaconstraint_assigner = MetaconstraintAssigner(environment, graph, [height_metaconstraint_builder])
    metaconstraint_assigner.assign_metaconstraints()

    """ Build a ConstraintAnalyzer and KeyframeGraphAnalyzer """
    constraint_analyzer = ConstraintAnalyzer(environment)
    graph_analyzer = KeyframeGraphAnalyzer(graph, moveit_interface, get_observation_joint_vector)

    sample_to_obsv_converter = SawyerSampleConverter(moveit_interface)
    sampler = KeyframeSampler(constraint_analyzer, sample_to_obsv_converter)

    """ Generate raw_samples from graph for each keyframe """
    for node in graph.get_keyframe_sequence():

        n_samples = args.number_of_samples
        constraints = [meta.constraints[4] for meta in graph.nodes[node]["metaconstraints"]]
        for constraint in constraints:
            print constraint
        attempts, samples, matched_ids = sampler.generate_n_valid_samples(graph.nodes[node]["model"], graph.nodes[node]["primal_observation"], constraints, n=n_samples)

        rospy.loginfo("Keyframe %d: %s valid of %s attempts", node, len(samples), attempts)
        if len(samples) < n_samples:
            rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(samples), n_samples)
        if len(samples) == 0:
            # TODO: DOWN SAMPLE METACONSTRAINTS AND KEEP TESTING
            rospy.loginfo("Keyframe %d has no valid sample observations", node)
            rospy.loginfo("Sampling with no meta constraints")
            attempts, samples, matched_ids = sampler.generate_n_valid_samples(graph.nodes[node]["model"], graph.nodes[node]["primal_observation"], [], n=n_samples)

        # Order sampled points based on their intra-model log-likelihood
        ranked_samples = sampler.rank_samples(graph.nodes[node]["model"], samples)

        # User converter object to convert raw sample vectors into LfD observations
        graph.nodes[node]["samples"] = [sample_to_obsv_converter.convert(sample, run_fk=True) for sample in ranked_samples]

    """ Clear occluded points (points in collision etc,.) """
    for node in graph.get_keyframe_sequence():
        samples = graph.nodes[node]["samples"]
        free_samples, trash = graph_analyzer.evaluate_keyframe_occlusion(samples)
        if free_samples == []:
            rospy.loginfo("Keyframe {} has no free samples and will be culled.".format(node))
            graph.cull_node(node)
        else:
            graph.nodes[node]["free_samples"] = free_samples

    """ Cull/remove keyframes/nodes that via change point estimation using log-likelihood """
    graph_analyzer.cull_keyframes(threshold=args.threshold)

    # """ Order sampled points based on their intra-model log-likelihood """
    # for node in graph.get_keyframe_sequence():
    #     graph.rank_waypoint_samples(node)

    output = []
    """ Create a sequence of keyframe way points and execute motion plans to reconstruct skill """
    joint_config_array = []
    for node in graph.get_keyframe_sequence():
        output.append((node, graph.nodes[node]["applied_constraints"]))
        sample = graph.nodes[node]["free_samples"][0]
        joints = sample.get_joint_angle()
        joint_config_array.append(joints)

    print output

    # moveit_interface.move_to_joint_targets(joint_config_array)

    return 0


if __name__ == '__main__':
    main()
