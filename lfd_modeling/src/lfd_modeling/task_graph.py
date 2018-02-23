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

import math

import rospy
import rospkg
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose

#TODO wrap these methods instead?
from sklearn.neighbors.kde import KernelDensity
from sklearn.mixture import GaussianMixture

import pdb

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
        self._head = 1
        for i in range(1, len(nodes)):
            self.add_edge(i, i+1)
        self._tail = len(nodes)+1

    def build_model(self, model="kde_gauss"):
        """
        create models with observations in node
        """
        for node in self.nodes():
            print node
            obsvs = self.nodes[node]['obsv']
            np_array = []
            for obsv in obsvs:
                robot = obsv.data["robot"]
                np_array.append(robot['position'] + robot['orientation'])
            np_array = np.array(np_array)

            if model == "kde_gauss":
                #TODO kernel density method is unwrapped change that
                #TODO load bandwidth from roslaunch params
                kde = KernelDensity(kernel='gaussian', bandwidth=.002).fit(np_array)
                self.nodes[node]['kde_gauss'] = kde
                rospy.loginfo("keyframe %s has build kde gaussian model", node)
            else:
                rospy.logwarn("No valid model created")

    def attribute_keyframe_type(self):
        for node in self.nodes():
            kf_id, kf_type = self.nodes[node]['obsv'][0].get_keyframe_info()
            self.nodes[node]["keyframe_type"] = kf_type

    def sample_n_obsv_objects(self, n, model):
        """
        wrapper for sampling points
        return obsv objects
        """
        samples = model.sample(n)
        obsv_array = []
        for sample in samples:
            normalize = math.sqrt(sample[3]**2 + sample[4]**2 +
                                  sample[5]**2 + sample[6]**2)
            sample[3] = sample[3]/normalize
            sample[4] = sample[4]/normalize
            sample[5] = sample[5]/normalize
            sample[6] = sample[6]/normalize
            obsv_array.append(Observation.init_samples(sample[0:3],
                                                       sample[3:7]))
        return obsv_array


    def sample_n_valid_waypoints(self, node_num, n = 100, model = "kde_gauss"):
        """
        returns a number of keypoints that are valid based on the constraints
        TODO messy and want to clean up
        only works with d method
        """
        if model != "kde_gauss":
            rospy.logerr("warning sampling valid waypoints only accepts kde_gauss models")
            return 0

        node = self.nodes[node_num]
        constraint_ids = node["obsv"][-1].data["applied_constraints"]
        model = node[model]

        yagami = DeathNote()
        valid_sample_obsv = []
        attempts = 0
        while len(valid_sample_obsv) < n:
            attempts += 1
            if attempts >= n*20:
                break
            sample = self.sample_n_obsv_objects(1, model)[0]
            matched_ids = self.analyzer._evaluator(constraint_ids, sample)
            if constraint_ids == matched_ids:
                valid_sample_obsv.append(sample)

        rospy.loginfo("%s valid of %s attempts", len(valid_sample_obsv), attempts)
        if len(valid_sample_obsv) < n:
            rospy.logwarn("only %s of %s waypoints provided", len(valid_sample_obsv), n)

        #TODO append method for adding more points?
        self.nodes[node_num]['samples'] = valid_sample_obsv
        return 0

    def rank_waypoint_samples(self, node_num, model="kde_gauss"):
        """
        re arrange all sampled points based on Prob Density
        """
        #TODO add desnity values to samples?
        if model != "kde_gauss":
            rospy.logerr("warning ranking valid waypoints only accepts kde_gauss models")
            return 0

        model = self.nodes[node_num][model]
        samples = self.nodes[node_num]['samples']

        np_array = []
        for sample in samples:
            robot = sample.data["robot"]
            np_array.append(np.concatenate([robot["position"], robot["orientation"]]))

        # pdb.set_trace()
        scores = model.score_samples(np_array)
        order = np.argsort(-scores)
        scores = scores[order]
        samples = np.asarray(samples)
        self.nodes[node_num]['samples'] = samples
        rospy.loginfo("keyframe %s samples have been reorderd", node_num)
        return 0

    def rank_waypoint_free_samples(self, node_num, model="kde_gauss"):
        """
        re arrange all sampled points based on Prob Density
        """
        #TODO add desnity values to samples?
        if model != "kde_gauss":
            rospy.logerr("warning ranking valid waypoints only accepts kde_gauss models")
            return 0

        model = self.nodes[node_num][model]
        samples = self.nodes[node_num]['free_samples']

        np_array = []
        for sample in samples:
            robot = sample.data["robot"]
            np_array.append(np.concatenate([robot["position"], robot["orientation"]]))

        # pdb.set_trace()
        scores = model.score_samples(np_array)
        order = np.argsort(-scores)
        scores = scores[order]
        samples = np.asarray(samples)
        self.nodes[node_num]['free_samples'] = samples
        rospy.loginfo("keyframe %s samples have been reorderd", node_num)
        return 0




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
    print "bad main old methods"
    return 0


if __name__ == '__main__':
    main()
