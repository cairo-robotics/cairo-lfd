#!/usr/bin/env python


import networkx as nx
from keyframe.modeling import GaussianMixtureModel
from keyframe.data_processing import DataImporter, DataProcessor

import os, signal

import rospy
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose
#from keyframe.data_processing import DataImporter, DataProcessor

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

Wait_flag = 0

def callback(string):
    global Wait_flag
    Wait_flag = 0
    print "completion callback"

def node_to_pose(pose_data):
    pose_msg = Pose()
    pose_msg.orientation.z = pose_data[0][3]
    pose_msg.orientation.x = pose_data[0][4]
    pose_msg.orientation.y = pose_data[0][5]
    pose_msg.orientation.w = pose_data[0][6]
    pose_msg.position.x = pose_data[0][0]
    pose_msg.position.y = pose_data[0][1]
    pose_msg.position.z = pose_data[0][2]
    return pose_msg

class TaskGraph(object):

    def __init__(self):
        self.task_graph = nx.DiGraph()
        self._processor = DataProcessor()
        self._head = None
        self._tail = None


    def add_node(self, samples):
        observations = []
        for observation in samples["data"]:
            sample = self._processor.convert_observation_dict_to_list(observation)
            observations.append(sample)

        np_observations = self._processor.to_np_array(observations)
        model = GaussianMixtureModel(np_observations)
        model.gmm_fit()


        new_pose = None
        i = 0
        while new_pose is None:
            test_pose = model.generate_samples(1)
            prior = model.predict_proba(test_pose)
            if any(P > .95 for P in prior[0][:]):
                pass
            else:
                new_pose = test_pose




        if self._head is None:
            self.task_graph.add_node(0, gmm=model, pose=new_pose)
            self._head = 0
            self._tail = 0
        else:
            self._tail += 1
            self.task_graph.add_node(self._tail, gmm=model, pose=new_pose)
            self.task_graph.add_edge(self._tail-1, self._tail)



    def check_sample_points(self, point, gmm):
        pass

def main():

    pose_pub = rospy.Publisher("/commander/pose", Pose, queue_size=1)
    sub_message = rospy.Subscriber("/test_topic", String, callback, queue_size=1)
    rospy.init_node("graph_traverse")
    print "hello world"
    yagami = DeathNote()

    task_graph = TaskGraph()

    importer = DataImporter()
    processor = DataProcessor()
    dir = os.path.dirname(__file__)
    filename = os.path.join(dir, "../../../lfd_keyframe_examples/toy_data/keyframe_data/keyframes.json")
    keyframe_data = importer.load_json_files(filename)

    for key in sorted(keyframe_data.keys()):
        task_graph.add_node(keyframe_data[key])


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




    print

if __name__ == '__main__':
    main()
