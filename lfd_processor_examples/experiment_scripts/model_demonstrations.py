#!/usr/bin/env python

import argparse
import rospy

from sawyer_interface.moveit_interface import SawyerMoveitInterface

from lfd_modeling.task_graph import TaskGraph
from lfd_processor.environment import Demonstration, Observation, Environment, import_configuration
from lfd_processor.data_io import DataImporter
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import TaskGraphAnalyzer, get_observation_joint_vector


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
        '-b', '--bandwidth', type=float, default=.008, metavar='BANDWIDTH',
        help='gaussian kernel density bandwidth'
    )

    parser.add_argument(
        '-t', '--threshold', type=int, default=-1000, metavar='THRESHOLD',
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

    """ Create TaskGraph object. """
    task_graph = TaskGraph(environment, moveit_interface, get_observation_joint_vector)

    """ Using labeled observations, build the models, graphs, and atributes of the TaskGraph """
    obsv_objects = []
    for demo in demonstrations:
        for ob in demo.observations:
            obsv_objects.append(ob)
    task_graph.add_obsvs_to_graph(obsv_objects)
    task_graph.link_graph()
    task_graph.build_model(bandwidth=args.bandwidth)
    task_graph.attribute_keyframe_type()

    """ Build a TaskGraphAnalyzer """
    task_graph_analyzer = TaskGraphAnalyzer(task_graph, moveit_interface, get_observation_joint_vector)

    """ Generate samples from graph """
    for node in task_graph.get_keyframe_sequence():
        # Sample point according to constraints
        task_graph.sample_n_valid_waypoints(node, n=args.number_of_samples, run_fk=True)
        # Sample points ignoring constraints:
        # task_graph.sample_n_waypoints(node, n=args.number_of_samples, run_fk=True)

    """ Clear occluded points (points in collision etc,.) """
    for node in task_graph.get_keyframe_sequence():
        samples = task_graph.nodes[node]["samples"]
        free_samples, trash = task_graph_analyzer.evaluate_keyframe_occlusion(samples)
        if free_samples == []:
            task_graph.cull_node(node)
        else:
            task_graph.nodes[node]["free_samples"] = free_samples

    """ Cull/remove keyframes/nodes that via change point estimation using log-liklihood """
    task_graph_analyzer.keyframe_culler(threshold=args.threshold)

    """ Order sampled points based on their intramodel log-liklihood """
    for node in task_graph.get_keyframe_sequence():
        task_graph.rank_waypoint_samples(node)

    rospy.loginfo(task_graph.get_keyframe_sequence())

    """ Create a seequence of keyframe waypoints and excute motion plans to reconstruct skill """
    joint_config_array = []
    for node in task_graph.get_keyframe_sequence():
        sample = task_graph.nodes[node]["free_samples"][0]
        joints = sample.get_joint_list()
        joint_config_array.append(joints)

    moveit_interface.move_to_joint_targets(joint_config_array)

    return 0


if __name__ == '__main__':
    main()
