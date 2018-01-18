#!/usr/bin/env python2.7

from keyframe.modeling import GaussianMixtureModel
from keyframe.data_processing import DataImporter, DataProcessor
from keyframe.visualization import SamplePointViewer, GaussianMixtureModelViewer
import os, signal

import networkx as nx

import rospy
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose

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
    pose_msg.orientation.w = pose_data[0][3]
    pose_msg.orientation.x = pose_data[0][4]
    pose_msg.orientation.y = pose_data[0][5]
    pose_msg.orientation.z = pose_data[0][6]
    pose_msg.position.x = pose_data[0][0]
    pose_msg.position.y = pose_data[0][1]
    pose_msg.position.z = pose_data[0][2]
    return pose_msg


def main():
    """
    An example of generating and visualizing GMM on the generated keyframes.json file from the toy data.
    """
    pose_pub = rospy.Publisher("/commander/pose", Pose, queue_size=1)
    sub_message = rospy.Subscriber("/test_topic", String, callback, queue_size=1)
    rospy.init_node("graph_traverse")

    yagami = DeathNote()

    importer = DataImporter()
    processor = DataProcessor()
    dir = os.path.dirname(__file__)
    filename = os.path.join(dir, "../../lfd_keyframe_examples/toy_data/keyframe_data/keyframes.json")
    keyframe_data = importer.load_json_files(filename)

    graph = nx.DiGraph()
    i = 0
    for key in sorted(keyframe_data.keys()):

        observations = []
        for observation in keyframe_data[key]["data"]:
            sample = processor.convert_observation_dict_to_list(observation)
            observations.append(sample)
        np_observations = processor.to_np_array(observations)

        model = GaussianMixtureModel(np_observations)
        model.gmm_fit()

        simulated_samples = model.generate_samples(1)
        graph.add_node(i, pose=simulated_samples)
        if i > 0:
            graph.add_edge(i-1, i)
        i = i + 1

    print i
    pose_data = Pose()
    current_node = 0

    pose_data = node_to_pose(graph.nodes[current_node]["pose"])
    print pose_data
    pose_pub.publish(pose_data)
    global Wait_flag
    Wait_flag = 1

    while list(graph.successors(current_node)):
        if yagami.write_name: #execute death note
            break
        temp = list(graph.successors(current_node))
        current_node = temp[0]
        print current_node
        while(Wait_flag):
            if yagami.write_name: #execute death note
                break
            pass
        Wait_flag = 1
        pose_data = node_to_pose(graph.nodes[current_node]["pose"])
        print pose_data
        pose_pub.publish(pose_data)




    while(True):
        if yagami.write_name: #execute death note
            break
    print "Hey, all it did was go around in a full circle."




if __name__ == "__main__":

    main()
