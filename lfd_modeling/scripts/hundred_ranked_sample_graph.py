#!/usr/bin/env python

from lfd_modeling.task_graph import TaskGraph
from lfd_processor.environment import Observation
from lfd_processor.data_io import DataImporter

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

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('lfd_processor_examples')

    file_path = "/toy_data/labeled_demonstrations/"
    traj_files = glob.glob(pkg_path + file_path + "*.json")


    obsv_objects = []
    #create array of trajectory objects
    for traj_file in traj_files:
        trajectories = importer.import_json_to_dict(traj_file)["trajectories"][0]
        for obsv in trajectories:
            obsv_objects.append(Observation(obsv))
    #build the graph
    task_graph.add_obsvs_to_graph(obsv_objects)
    task_graph.link_graph()
    task_graph.build_model()


    #create ranked samples as attribute for graph
    for node in task_graph.nodes():
        task_graph.sample_n_valid_waypoints(node)
        task_graph.rank_waypoint_samples(node)
