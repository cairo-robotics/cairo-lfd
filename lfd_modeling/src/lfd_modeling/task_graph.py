#!/usr/bin/env python
import os, signal
import numpy as np
import networkx as nx
from networkx import MultiDiGraph
import itertools

from lfd_modeling.modeling import GaussianMixtureModel
from lfd_processor.data_io import DataImporter
from lfd_processor.environment import Environment, import_configuration, Observation
from lfd_processor.items import RobotFactory, ConstraintFactory
from lfd_processor.analyzer import ConstraintAnalyzer
from sawyer_interface.moveit_interface import SawyerMoveitInterface
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
    """ For creating a graphical representation of a task
    Inherited from the MultiDiGraph graph from networkx

    Attributes:
        _head: the first node of the graph
        _tail: the last node of the graph
        enviroment: object containg the applied contraints for task
        analyzer: object that analyzes Observation objects for constaint violations
        MultiDiGraph: the inherited graph object

    Node Attributes:
        obsvs: list of observation objects the node uses to create its model
        kde_gauss: the kernel Gaussian model of the obsvs sklean KernelDenisty
        gmm: Gaussian Mixture Model of obsvs wrapped class of sklearn GMM
        samples: List of observation objects containing the samples
    """
    def __init__(self, environment, sawyer_moveit_interface, observation_vectorizor):
        MultiDiGraph.__init__(self)
        self._head = None
        self._tail = None

        self.environment = environment
        self.analyzer = ConstraintAnalyzer(self.environment)
        self.interface = sawyer_moveit_interface
        self.vectorizor = observation_vectorizor

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
        # Remove the nodes that didn't get any observations.
        for node in self.nodes():
            if 'obsv' not in self.nodes[node].keys():
                self.cull_node(node)

    def build_model(self, model="kde_gauss", bandwidth=.001):
        """
        create models with observations in node
        """
        for node in self.nodes():
            if 'obsv' in self.nodes[node].keys():
                obsvs = self.nodes[node]['obsv']
                np_array = []
                for obsv in obsvs:
                    np_array.append(np.array(self.vectorizor(obsv)))
                np_array = np.array(np_array)
                if model == "kde_gauss":
                    #TODO kernel density method is unwrapped change that
                    #TODO load bandwidth from roslaunch params
                    kde = KernelDensity(kernel='gaussian', bandwidth=bandwidth).fit(np_array)
                    self.nodes[node]['kde_gauss'] = kde
                    rospy.loginfo("keyframe %s has build kde gaussian model", node)
                else:
                    rospy.logwarn("No valid model created")
            else:
                rospy.logwarn("No observations for keyframe ID: {}".format(node))


    def attribute_keyframe_type(self):
        for node in self.nodes():
            if 'obsv' in  self.nodes[node].keys():
                kf_id, kf_type = self.nodes[node]['obsv'][0].get_keyframe_info()
                self.nodes[node]["keyframe_type"] = kf_type

    def sample_n_obsv_objects(self, n, model, run_fk = False):
        """
        wrapper for sampling points
        return obsv objects
        """
        samples = model.sample(n)

        if run_fk is True:
            new_samples = []
            for sample in samples:
                pose = self.interface.get_end_effector_pose(sample.tolist())
                if pose is not None:
                    sample = np.insert(sample, 0, pose.orientation.w, axis=0)
                    sample = np.insert(sample, 0, pose.orientation.z, axis=0)
                    sample = np.insert(sample, 0, pose.orientation.y, axis=0)
                    sample = np.insert(sample, 0, pose.orientation.x, axis=0)
                    sample = np.insert(sample, 0, pose.position.z, axis=0)
                    sample = np.insert(sample, 0, pose.position.y, axis=0)
                    sample = np.insert(sample, 0, pose.position.x, axis=0)
                    new_samples.append(sample)
            samples = new_samples

        obsv_array = []
        for sample in samples:
            normalize = math.sqrt(sample[3]**2 + sample[4]**2 +
                                  sample[5]**2 + sample[6]**2)
            sample[3] = sample[3]/normalize
            sample[4] = sample[4]/normalize
            sample[5] = sample[5]/normalize
            sample[6] = sample[6]/normalize

            if len(sample) > 7:
                # If length > 7, we know there must be joint data, so creat Obs w/ joints.
                obsv = Observation.init_samples(sample[0:3], sample[3:7], sample[7:14])
            else:
                obsv = Observation.init_samples(sample[0:3], sample[3:7], None)
            obsv_array.append(obsv)
        return obsv_array


    def sample_n_valid_waypoints(self, node_num, n = 100, model = "kde_gauss", run_fk = False):
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

        valid_sample_obsv = []
        attempts = 0
        while len(valid_sample_obsv) < n:
            attempts += 1
            if attempts >= n*20:
                break
            samples = self.sample_n_obsv_objects(1, model, run_fk=run_fk)
            if len(samples) > 0:
                sample = samples[0]
                matched_ids = self.analyzer._evaluator(constraint_ids, sample)
                if constraint_ids == matched_ids:
                    valid_sample_obsv.append(sample)

        rospy.loginfo("%s valid of %s attempts", len(valid_sample_obsv), attempts)
        if len(valid_sample_obsv) < n:
            rospy.logwarn("only %s of %s waypoints provided", len(valid_sample_obsv), n)
        if len(valid_sample_obsv) == 0:
            rospy.loginfo("Node {} has no valid sample observations".format(node_num))
            self.cull_node(node_num)


        #TODO append method for adding more points?
        self.nodes[node_num]['samples'] = valid_sample_obsv
        return 0

    def sample_n_waypoints(self, node_num, n = 100, model = "kde_gauss", run_fk = False):
        if model != "kde_gauss":
            rospy.logerr("warning sampling valid waypoints only accepts kde_gauss models")
            return 0
        node = self.nodes[node_num]
        model = node[model]

        self.nodes[node_num]['samples'] = self.sample_n_obsv_objects(n, model, run_fk=run_fk)
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
            np_array.append(np.array(self.vectorizor(sample)))

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
            np_array.append(np.array(self.vectorizor(sample)))

        # pdb.set_trace()
        scores = model.score_samples(np_array)
        order = np.argsort(-scores)
        scores = scores[order]
        samples = np.asarray(samples)
        self.nodes[node_num]['free_samples'] = samples
        rospy.loginfo("keyframe %s samples have been reorderd", node_num)
        return 0

    def cull_node(self, node):
        """
        takes received node and removes it from the task graph
        Parameters
        ----------
        node: hashable networkx node name
            the node to be removed from network x graph

        Returns
        -------
        -1 for failure
        0 for success

        """
        
        next_nodes = [x for x in self.successors(node)]
        prev_nodes = [x for x in self.predecessors(node)]
        if next_nodes == []:
            prev_node = prev_nodes[0]
            self.remove_edge(prev_node, node)
            rospy.loginfo("Node %s has been culled", node)
        elif prev_nodes == []:
            next_node = next_nodes[0]
            self.remove_edge(node, next_node)
            rospy.loginfo("Node %s has been culled", node)
        else:
            prev_node = prev_nodes[0]
            next_node = next_nodes[0]
            self.add_edge(prev_node, next_node)
            self.remove_edge(prev_node, node)
            self.remove_edge(node, next_node)
            rospy.loginfo("Node %s has been culled", node)
        return 0

    def get_keyframe_sequence(self):
        node_chain = list(set(itertools.chain(*self.edges())))
        node_chain.sort()
        return node_chain

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
