#!/usr/bin/env python

import os, signal
import numpy as np
import networkx as nx

from lfd_modeling.task_graph import TaskGraph
from lfd_processor.data_io import DataImporter
from lfd_processor.environment import Environment, import_configuration

import rospy
import rospkg
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose




def main():
    pose_pub = rospy.Publisher("/commander/pose", Pose, queue_size=1)
    rospy.init_node("graph_traverse")
    print "hello world"

    '''build constraint enviroment'''

    rospack = rospkg.RosPack()
    pkg_dir = rospack.get_path('lfd_processor_examples')
    config_loc = '/toy_data/configurations/'
    config_name = 'config.json'

    filepath = pkg_dir + config_loc + config_name

    configs = import_configuration(filepath)


    return 0
    ''' build graph '''
    task_graph = TaskGraph()
    importer = DataImporter()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('lfd_processor_examples')

    file_path = "/toy_data/labeled_demonstrations/"
    traj_file = "labeled_demonstration0.json"
    keyframe_data = importer.import_json_to_dict(pkg_path + file_path+ traj_file)
    keyframe_data = keyframe_data["trajectories"][0]

    task_graph.builder(keyframe_data)






'''
    sample =  task_graph.sample_n_from_node(1, 1)
    pose_msg = Pose()
    sample = sample[0]
    print sample

    pose_msg.position.x = sample[0]
    pose_msg.position.y = sample[1]
    pose_msg.position.z = sample[2]
    pose_msg.orientation.x = sample[3]
    pose_msg.orientation.y = sample[4]
    pose_msg.orientation.z = sample[5]
    pose_msg.orientation.w = sample[6]

    pose_pub.publish(pose_msg)
'''


if __name__ == '__main__':
    main()
