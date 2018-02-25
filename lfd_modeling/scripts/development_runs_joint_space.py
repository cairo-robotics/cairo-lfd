#!/usr/bin/env python

from sawyer_interface.moveit_interface import  SawyerMoveitInterface

from lfd_modeling.task_graph import TaskGraph
from lfd_processor.environment import Observation, Environment, import_configuration
from lfd_processor.data_io import DataImporter
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import ConstraintAnalyzer, MotionPlanAnalyzer, TaskGraphAnalyzer, get_observation_joint_vector

import geometry_msgs.msg
from geometry_msgs.msg import Pose

import itertools
import glob
import rospy
import rospkg
import pdb


def main():
    """
    creates task tree graph with 100 ranked observations
    uses the example data found in lfd_processor_examples package
    must have labled demonstrations from scripts/keyframe_labeling_example.py
    or similar script
    """

    rospy.init_node("graph_traverse")
    print "hello world"


    '''constraint file path'''
    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('lfd_processor_examples')
    config_loc = '/toy_data/configurations/'
    config_name = 'config.json'

    config_filepath = pkg_dir + config_loc + config_name


    '''create analyzers for plan evaluation'''
    configs = import_configuration(config_filepath)
    robot_factory = RobotFactory(configs["robots"])
    constraint_factory = ConstraintFactory(configs["constraints"])

    robot = robot_factory.generate_robots()[0]
    constraints = constraint_factory.generate_constraints()
    environment = Environment(items=None, robot=robot,
                              constraints=constraints)
    #analyzer = ConstraintAnalyzer(environment)

    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.35)
    moveit_interface.set_acceleration_scaling(.35)

    motion_plan_analyzer = MotionPlanAnalyzer(environment)
    start_joint1 = [0.315572265625, -0.0999033203125, 3.0444072265625, -1.0162578125, 0.363646484375, 0.8749013671875, 1.6634150390625]
    start_joint2 = [-0.193619140625, 0.818009765625, -1.2855966796875, -1.3627841796875, -0.8378056640625, 1.1223134765625, 0.1806572265625]
    start_joint3 = [0.2708837890625, 0.4152705078125, -1.909638671875, -0.58271875, -0.046583984375, 1.2423203125, 0.7582158203125]
    #moveit_interface.move_to_joint_target(start_joint1)
    start_pose1 = [0.723175561368, 0.404886545639, 0.000902259841386, 0.723185938034, -0.00519365801061, 0.690262924376, -0.0226322817993 ]

    # moveit_interface.move_to_pose_target(start_pose1)

    ''' build graph '''
    task_graph = TaskGraph(environment, moveit_interface, get_observation_joint_vector)
    importer = DataImporter()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('lfd_processor_examples')

    file_path = "/toy_data/poor_demonstration_resilience/"
    traj_files = glob.glob(pkg_path + file_path + "*.json")

    obsv_objects = []
    #create array of trajectory objects
    for traj_file in traj_files:
        trajectories = importer.import_json_to_dict(traj_file)["trajectories"][0]
        for obsv in trajectories:
            obsv_objects.append(Observation(obsv))
    #build the graph
    task_graph.add_obsvs_to_graph(obsv_objects)
    task_graph.link_graph() #step where head and tail are set
    task_graph.build_model(bandwidth=.008)
    task_graph.attribute_keyframe_type()

    '''create analyzer adn motion interface'''

    task_graph_analyzer = TaskGraphAnalyzer(task_graph, moveit_interface, get_observation_joint_vector)
    '''full task graph model built all methods from here down iterate
    through the graph'''

    '''create samples from graph'''
    for node in task_graph.get_keyframe_sequence():
        # Sample point according to constraints
        task_graph.sample_n_valid_waypoints(node, n=20, run_fk=True)
        # Sample points ignoring constraints:
        # task_graph.sample_n_waypoints(node, n=20, run_fk=True)

    '''clear occluded points'''
    for node in task_graph.get_keyframe_sequence():
        samples = task_graph.nodes[node]["samples"]
        free_samples, trash = task_graph_analyzer.evaluate_keyframe_occlusion(samples)
        if free_samples == []:
            task_graph.cull_node(node)
        else:
            task_graph.nodes[node]["free_samples"] = free_samples

    ''' cull high liklihood nodes'''
    task_graph_analyzer.keyframe_culler(threshold=-1000)

    '''rank sampled points'''
    for node in task_graph.get_keyframe_sequence():
        task_graph.rank_waypoint_samples(node)

    rospy.loginfo(task_graph.get_keyframe_sequence())

    '''run through best waypoints'''
    #TODO super contrived method for iterating through nodes
    
    joint_config_array = []
    for node in task_graph.get_keyframe_sequence():
        sample = task_graph.nodes[node]["free_samples"][0]
        joints = sample.get_joint_list()
        joint_config_array.append(joints)

    moveit_interface.move_to_joint_targets(joint_config_array)
        #create ranked samples as attribute for graph

    return 0

if __name__ == '__main__':
    main()
