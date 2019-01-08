#!/usr/bin/env python

import argparse
import rospy

from robot_interface.moveit_interface import SawyerMoveitInterface

from lfd.graphing import ObservationClusterer, KeyframeGraph
from lfd.models import KDEModel
from lfd.sampling import KeyframeSampler
from lfd.environment import Demonstration, Observation, Environment, import_configuration
from lfd.data_io import DataImporter
from lfd.items import RobotFactory, ConstraintFactory
from lfd.analyzer import KeyframeGraphAnalyzer, ConstraintAnalyzer, get_observation_joint_vector
from lfd.processing import SawyerSampleConverter

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
        help='log-liklihood threshold value'
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
    robot_factory = RobotFactory(configs["robots"])
    constraint_factory = ConstraintFactory(configs["constraints"])
    robot = robot_factory.generate_robots()[0]
    constraints = constraint_factory.generate_constraints()
    environment = Environment(items=None, robot=robot,
                              constraints=constraints)

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
        graph.nodes[cluster_id]["model"] = KDEModel(kernel='gaussian', bandwidth=args.bandwidth)
    graph.add_path(graph.nodes())
    graph.fit_models(get_observation_joint_vector)

    """ Build a ConstraintAnalyzer and KeyframeGraphAnalyzer """
    constraint_analyzer = ConstraintAnalyzer(environment)
    graph_analyzer = KeyframeGraphAnalyzer(graph, moveit_interface, get_observation_joint_vector)

    sample_to_obsv_converter = SawyerSampleConverter(moveit_interface)
    sampler = KeyframeSampler(constraint_analyzer, sample_to_obsv_converter)

    """ Generate raw_samples from graph for each keyframe """
    for node in graph.get_keyframe_sequence():
        # Sample point according to constraints
        n_samples = args.number_of_samples
        attempts, samples = sampler.generate_n_valid_samples(graph.nodes[node]["model"], graph.nodes[node]["applied_constraints"], n=n_samples)

        rospy.loginfo("Keyframe %d: %s valid of %s attempts", node, len(samples), attempts)
        if len(samples) < n_samples:
            rospy.loginfo("Keyframe %d: only %s of %s waypoints provided", node, len(samples), n_samples)
        if len(samples) == 0:
            rospy.loginfo("Keyframe %d has no valid sample observations", node)
            graph.cull_node(node)
        # Order sampled points based on their intramodel log-liklihood
        ranked_samples = sampler.rank_samples(graph.nodes[node]["model"], samples)

        # User converter object to conver raw sample vectors into LfD observations
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

    """ Cull/remove keyframes/nodes that via change point estimation using log-liklihood """
    graph_analyzer.keyframe_culler(threshold=args.threshold)

    # """ Order sampled points based on their intramodel log-liklihood """
    # for node in graph.get_keyframe_sequence():
    #     graph.rank_waypoint_samples(node)

    rospy.loginfo(graph.get_keyframe_sequence())

    """ Create a seequence of keyframe waypoints and excute motion plans to reconstruct skill """
    joint_config_array = []
    for node in graph.get_keyframe_sequence():
        sample = graph.nodes[node]["free_samples"][0]
        joints = sample.get_joint_list()
        joint_config_array.append(joints)

    moveit_interface.move_to_joint_targets(joint_config_array)

    return 0


if __name__ == '__main__':
    main()
