#!/usr/bin/env python

import os, signal
import numpy as np
import networkx as nx

from lfd_modeling.task_graph import TaskGraph
from lfd_processor.data_io import DataImporter
from lfd_processor.environment import Environment, import_configuration, Observation
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import ConstraintAnalyzer

from lfd_modeling.visualization import SamplePointViewer

from pprint import PrettyPrinter
from json import loads, dumps
import glob

from sklearn.neighbors.kde import KernelDensity
from sklearn.mixture import GaussianMixture

import math
import rospy
import rospkg
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose

PRINT_STATE = 0
#TODO temporary wild loop killer
class DeathNote(object):
    '''DeathNote for exiting runaway loops'''
    write_name = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.ryuk)
        signal.signal(signal.SIGTERM, self.ryuk)

    def ryuk(self, signum, frame):
        '''name written kill the loop'''
        self.write_name = True




def main():
    yagami = DeathNote()

    pose_pub = rospy.Publisher("/commander/pose", Pose, queue_size=1)
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


    task_graph.add_obsvs_to_graph(obsv_objects)
    task_graph.link_graph()


    viewer = SamplePointViewer()
    pose_array = []

    for node in task_graph.nodes():
        print node
        obsvs = task_graph.nodes[node]['obsv']
        np_array = []
        for obsv in obsvs:
            robot = obsv.data["robot"]
            np_array.append(robot['position'] + robot['orientation'])

        np_array = np.array(np_array)
        kde = KernelDensity(kernel='gaussian', bandwidth=.1).fit(np_array)
        task_graph.nodes[node]['kde_gauss'] = kde

        samples = kde.sample(1000)
        task_graph.nodes[node]['samples'] = samples
        scores = kde.score_samples(samples)
        position = np.argmax(scores)
        pose_array.append(samples[position])
        #viewer.view_3D_scatter(samples, 0, 1, 2)
        print scores[position]
        #keypress = raw_input("press key to continue")
        #if keypress == "q":
        #    return 0

    #print pose_array




    pose_msg = Pose()
    i = 0
    for pose in pose_array:
        print obsv
        pose_msg.position.x = pose[0]
        pose_msg.position.y = pose[1]
        pose_msg.position.z = pose[2]

        normalize = math.sqrt(pose[3]**2 + pose[4]**2 + pose[5]**2 + pose[6]**2)
        pose_msg.orientation.x = pose[3]/normalize
        pose_msg.orientation.y = pose[4]/normalize
        pose_msg.orientation.z = pose[5]/normalize
        pose_msg.orientation.w = pose[6]/normalize

        pose_pub.publish(pose_msg)
        i += 1
        print i
        print pose_msg
        samples = task_graph.nodes[i]['samples']
        viewer.view_3D_scatter(samples, 0, 1, 2)
        keypress = raw_input("press key to continue")
        print keypress
        if keypress == "q":
            break

if __name__ == '__main__':
    main()
