#!/usr/bin/env python
import numpy as np
import networkx as nx
from networkx import MultiDiGraph

from lfd_modeling.modeling import GaussianMixtureModel
from lfd_processor.data_io import DataImporter

import os, signal

import rospy
import rospkg
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose

import os, signal


class DeathNote(object):
    '''DeathNote for exiting runaway loops'''
    write_name = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.ryuk)
        signal.signal(signal.SIGTERM, self.ryuk)

    def ryuk(self, signum, frame):
        '''name written kill the loop'''
        self.write_name = True



class TaskGraph(MultiDiGraph):

    def __init__(self):
        MultiDiGraph.__init__(self)
        #self._processor = DataProcessor()
        self._head = None
        self._tail = None


    def add_gmm_node(self, samples):
        pass
        '''
        observations = []
        for observation in samples["data"]:
            sample = self._processor.convert_observation_dict_to_list(observation)
            observations.append(sample)

        np_observations = self._processor.to_np_array(observations)
        model = GaussianMixtureModel(np_observations)
        model.gmm_fit()

        if self._head is None:
            self.add_node(0, gmm=model)
            self._head = 0
            self._tail = 0
        else:
            self._tail += 1
            self.add_node(self._tail, gmm=model)
            self.add_edge(self._tail-1, self._tail)
        '''


    def add_n_samples_to_node(self, n, node):
        pass
        '''
        sample = self.nodes[ node]['gmm'].generate_samples(1)
        self.nodes[node]['points'].append(sample)
        '''


    def check_sample_points(self, point, gmm):
        pass
        '''
        new_pose = None
        i = 0
        while new_pose is None:
            pass
            if any(P > .95 for P in prior[0][:]):
                pass
            else:
                new_pose = test_pose
                '''

def main():
    yagami = DeathNote()

    pose_pub = rospy.Publisher("/commander/pose", Pose, queue_size=1)
    rospy.init_node("graph_traverse")
    print "hello world"

    task_graph = TaskGraph()
    importer = DataImporter()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('lfd_processor')

    file_path = "/src/lfd_processor/"
    traj_file = "trajectory2.json"




    keyframe_data = importer.import_json_to_dict(pkg_path + file_path+ traj_file)

    for key in keyframe_data["trajectories"]:
        print len(key)

    print keyframe_data["trajectories"][0][0]["time"]

    '''
    for key in sorted(keyframe_data.keys()):
        task_graph.add_gmm_node(keyframe_data[key])

    task_graph.add_sample_to_node(1)

    print task_graph.nodes[1]['points']

    global Wait_flag
    Wait_flag = 1
    current_node = 0

    pose_data = node_to_pose(task_graph.task_graph.nodes[current_node]["pose"])

    print pose_data
    pose_pub.publish(pose_data)
    global Wait_flag
    Wait_flag = 1

    while list(task_graph.task_graph.successors(current_node)):
        if yagami.write_name: #execute death note
            break
        temp = list(task_graph.task_graph.successors(current_node))
        current_node = temp[0]
        print current_node
        while(Wait_flag):
            if yagami.write_name: #execute death note
                break
            pass
        Wait_flag = 1
        pose_data = node_to_pose(task_graph.task_graph.nodes[current_node]["pose"])
        print pose_data
        pose_pub.publish(pose_data)

    #print prob

    '''


    print

if __name__ == '__main__':
    main()
