#!/usr/bin/env python
import os, signal
import numpy as np
import networkx as nx
from networkx import MultiDiGraph

from lfd_modeling.modeling import GaussianMixtureModel
from lfd_processor.data_io import DataImporter
from lfd_processor.environment import Environment, import_configuration, Observation
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import ConstraintAnalyzer

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
    def __init__(self, config_path):
        MultiDiGraph.__init__(self)
        self._head = None
        self._tail = None

        configs = import_configuration(config_path)
        robot_factory = RobotFactory(configs["robots"])
        constraint_factory = ConstraintFactory(configs["constraints"])

        robot = robot_factory.generate_robots()[0]
        constraints = constraint_factory.generate_constraints()

        self.environment = Environment(items=None, robot=robot,
                                       constraints=constraints)
        self.analyzer = ConstraintAnalyzer(self.environment)


    def add_obsvs_to_graph(self, obsv_objects):
        """
        TODO finish making function definition
        """
        empty_obsvs = 0
        positive_obsvs = 0
        for obsv in obsv_objects:
            keyframe_num = obsv.data["keyframe_id"]
            if keyframe_num is None:
                empty_obsvs += 1
            else:
                self.add_obsv_to_node(obsv)
                positive_obsvs += 1

        nodes = self.nodes
        for node in nodes:
            rospy.loginfo(" %s observations in keyframe %s",
                          len(self.node[node]['obsv']), node)
        rospy.loginfo("%s observations discarded of %s", empty_obsvs,
                      empty_obsvs + positive_obsvs)


    def add_obsv_to_node(self, obsv):
        """
        add obsv to node
        """
        if obsv.data["keyframe_id"] in self.nodes():
            observation_array = self.node[obsv.data["keyframe_id"]]["obsv"]
            observation_array.append(obsv)
            self.node[obsv.data["keyframe_id"]]["obsv"] = observation_array
        else:
            observation = []
            observation.append(obsv)
            self.add_node(obsv.data["keyframe_id"], obsv=observation)

    def link_graph(self):
        """
        link keyframes nodes together with edges
        """
        nodes = self.nodes
        for i in range(1, len(nodes)):
            self.add_edge(i, i+1)


    def sample_for_valid_waypoints(self, node_num, n):
        """
        works yay! TODO define function and comment code
        break down into smaller functions?
        add overal attempts number to be logged
        """
        node = self.nodes[node_num]
        constraint_ids = node["obsv"][-1].data["applied_constraints"]
        gmm_model = node["gmm"]

        i = 0
        waypoint_array = []
        while len(waypoint_array) < n:
            i += 1
            j = 0
            while True:
                j += 1
                sample_array = gmm_model.generate_samples(1).tolist()[0]
                sample_obsv = Observation.init_samples(sample_array[0:3],
                                                       sample_array[3:7])

                matched_ids = self.analyzer.evaluator(constraint_ids,
                                                      sample_obsv)

                if constraint_ids == matched_ids:
                    break
                if j > 200:
                    rospy.logwarn("200 samples no valid waypoint")
                    break
            waypoint_array.append(sample_obsv)
            if i > (n*2):
                rospy.logerr("%swaypoint attempts failed for node %s", n, node_num)
                break
        return waypoint_array

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
