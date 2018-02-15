#!/usr/bin/env python
import os, signal
import numpy as np
import networkx as nx
from networkx import MultiDiGraph

from lfd_modeling.modeling import GaussianMixtureModel
from lfd_processor.data_io import DataImporter

import rospy
import rospkg
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose


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



class TaskGraph(MultiDiGraph):
    """
    TODO add class defenition
    """
    def __init__(self):
        MultiDiGraph.__init__(self)
        self._head = None
        self._tail = None

    def builder(self, keyframe_data):
        """
        TODO finish making function definition
        """
        # prototype task graph builder
        observations = []
        if keyframe_data[0]["keyframe_id"] is None:
            rospy.loginfo("first data point no keyframe")
        keyframe_num = 1

        for data in keyframe_data:
            #no keyframe discard
            if data["keyframe_id"] is None:
                pass
            else:
                #new keyframe create new graph node
                if data["keyframe_id"] == keyframe_num + 1:
                    rospy.loginfo("%s data points in keyframe %s",
                                  len(observations), keyframe_num)
                    keyframe_num += 1
                    self.add_gmm_node(observations)
                    #clear and append new data
                    observations = []
                    observations.append(data)
                #same keyframe append data
                elif data["keyframe_id"] == keyframe_num:
                    observations.append(data)
                #uhoh weird mismatch
                else:
                    rospy.logerr("keyframe_id mismatch %s on %s",
                                 data["keyframe_id"], keyframe_num)

        #create final keyframe
        rospy.loginfo("%s data points in keyframe %s",
                      len(observations), keyframe_num)
        self.add_gmm_node(observations)

        if keyframe_data[-1]["keyframe_id"] is None:
            rospy.loginfo("last data point no keyframe")


    def add_gmm_node(self, observations):
        """
        TODO add functin definition
        """
        #TODO try without np array conversion?
        #does it matter? doesn't numpy just wrap on existing array
        np_poses = []
        for obsrv in observations:
            robot = obsrv["robot"]
            np_poses.append(robot["position"] + robot["orientation"])
        np_poses = np.array(np_poses)

        #create and fit model
        model = GaussianMixtureModel(np_poses)
        model.gmm_fit()

        if self._head is None:
            self.add_node(0, gmm=model)
            self._head = 0
            self._tail = 0
        else:
            self._tail += 1
            self.add_node(self._tail, gmm=model)
            self.add_edge(self._tail-1, self._tail)
        rospy.loginfo("node %s added to task graph", self._tail)



    def sample_n_from_node(self, n, node):
        """
        TODO fully define the function
        """
        samples = self.nodes[node]['gmm'].generate_samples(n)

        return samples



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
    #TODO remove when module is done
    yagami = DeathNote()

    pose_pub = rospy.Publisher("/commander/pose", Pose, queue_size=1)
    rospy.init_node("graph_traverse")
    print "hello world"

    task_graph = TaskGraph()
    importer = DataImporter()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('lfd_processor_examples')

    file_path = "/toy_data/labeled_demonstrations/"
    traj_file = "labeled_demonstration0.json"
    keyframe_data = importer.import_json_to_dict(pkg_path + file_path+ traj_file)
    keyframe_data = keyframe_data["trajectories"][0]

    task_graph.builder(keyframe_data)

    print task_graph.nodes

    sample =  task_graph.sample_n_from_node(1, 5)
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


if __name__ == '__main__':
    main()
