#!/usr/bin/env python
import random
import itertools
import numpy as np

from lfd_modeling.task_graph import TaskGraph
from lfd_processor.environment import Observation
from lfd_processor.data_io import DataImporter
from lfd_processor.analyzer import TaskGraphAnalyzer
from sawyer_interface.moveit_interface import SawyerMoveitInterface

import glob
import rospy
import rospkg

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

    ''' build graph '''
    task_graph = TaskGraph(config_filepath)
    importer = DataImporter()
    interface = SawyerMoveitInterface()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('lfd_processor_examples')

    file_path = "/toy_data/full_system_test/"
    traj_files = glob.glob(pkg_path + file_path + "*.json")

    tg_analyzer = TaskGraphAnalyzer(task_graph, interface)

    obsv_objects = []
    # create array of trajectory objects
    for traj_file in traj_files:
        trajectories = importer.import_json_to_dict(traj_file)["trajectories"][0]
        for obsv in trajectories:
            obsv_objects.append(Observation(obsv))
    # build the graph
    task_graph.add_obsvs_to_graph(obsv_objects)
    task_graph.link_graph()
    task_graph.build_model()
    task_graph.attribute_keyframe_type()

    # create ranked samples as attribute for graph
    for node in task_graph.nodes():
        task_graph.sample_n_valid_waypoints(node, n=50)
        task_graph.rank_waypoint_samples(node)


    before_nodes = list(set(itertools.chain(*task_graph.edges())))

    tg_analyzer.keyframe_culler()

    after_nodes = list(set(itertools.chain(*task_graph.edges())))



    node = task_graph._head
    for node in after_nodes:
        free, occluded = tg_analyzer.evaluate_keyframe_occlusion(task_graph.nodes[node]["samples"])
        task_graph.nodes[node]["free_samples"] = np.array(free)

    for node in after_nodes:
        if len(task_graph.nodes[node]["free_samples"]) > 0:
            task_graph.rank_waypoint_free_samples(node)

    poses = []
    for node in after_nodes:
        if len(task_graph.nodes[node]["free_samples"]) > 0:
            sample_ob = task_graph.nodes[node]["free_samples"][0]
            data = sample_ob.get_robot_data()
            position = list(data["position"])
            orientation = list(data["orientation"])
            pose_vector = position + orientation
            poses.append(pose_vector)
    
    print "Starting ask graph:"
    print before_nodes
    print "\n\nCulled task graph:"
    print after_nodes

    print "Number of keyframes to perform: {}".format(len(poses))
    interface.move_to_pose_targets(poses)

if __name__ == "__main__":
    main()