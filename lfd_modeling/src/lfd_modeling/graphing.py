import itertools
import math
from collections import defaultdict

from lfd_processor.environment import Observation

import rospy
import numpy as np
from networkx import MultiDiGraph
from sklearn.neighbors.kde import KernelDensity


class KeyframeGraph(MultiDiGraph):
    def __init__(self):
        MultiDiGraph.__init__(self)

    def get_keyframe_sequence(self):
        node_chain = list(set(itertools.chain(*self.edges())))
        node_chain.sort()
        return node_chain

    def cull_node(self, node):
        """
        takes received node and removes it from the task graph
        Parameters
        ----------
        node: hashable networkx node name
            the node to be removed from network x graph

        Returns
        -------
        0 for failure
        1 for success

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
        return 1

    def fit_models(self, bandwidth=.001, observation_vectorizor):
        """
        create models with observations in node
        """
        for node in self.nodes():
            np_array = []
            for obsv in self[node]["observations"]:
                np_array.append(np.array(observation_vectorizor(obsv)))
            self[node]["model"].fit(np_array)


class KeyframeSampler():

    def __init__(self, robot_interface, analyzer, graph):
        self.interface = robot_interface
        self.analyzer = analyzer
        self.graph = graph

    def sample_n_observations(self, model, num_of_samples, run_fk=False):
        """
        wrapper for sampling points
        return obsv objects
        """
        samples = model.sample(num_of_samples)

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
            sample[3] = sample[3] / normalize
            sample[4] = sample[4] / normalize
            sample[5] = sample[5] / normalize
            sample[6] = sample[6] / normalize

            if len(sample) > 7:
                # If length > 7, we know there must be joint data, so creat Obs w/ joints.
                obsv = Observation.init_samples(sample[0:3], sample[3:7], sample[7:14])
            else:
                obsv = Observation.init_samples(sample[0:3], sample[3:7], None)
            obsv_array.append(obsv)
        return obsv_array

    def sample_n_valid_waypoints(self, model, node_id, n=100, run_fk=False):
        """
        returns a number of keypoints that are valid based on the constraints
        TODO messy and want to clean up
        only works with d method
        """

        node = self.nodes[node_num]
        constraint_ids = node["obsv"][-1].data["applied_constraints"]
        model = node[model]

        valid_sample_obsv = []
        attempts = 0
        while len(valid_sample_obsv) < n:
            attempts += 1
            if attempts >= n * 20:
                break
            samples = self.sample_n_observations(1, model, run_fk=run_fk)
            if len(samples) > 0:
                sample = samples[0]
                matched_ids = self.analyzer._evaluator(constraint_ids, sample)
                if constraint_ids == matched_ids:
                    valid_sample_obsv.append(sample)

        rospy.loginfo("%s valid of %s attempts", len(valid_sample_obsv), attempts)
        if len(valid_sample_obsv) < n:
            rospy.logwarn("only %s of %s waypoints provided", len(valid_sample_obsv), n)
        if len(valid_sample_obsv) == 0:
            rospy.loginfo("Node {} has no valid sample observations".format(node_id))
            self.cull_node(node_num)

        # TODO append method for adding more points?
        self.nodes[node_num]['samples'] = valid_sample_obsv
        return 0

    def sample_n_waypoints(self, node_num, n=100, model="kde_gauss", run_fk=False):
        if model != "kde_gauss":
            rospy.logerr("warning sampling valid waypoints only accepts kde_gauss models")
            return 0
        node = self.nodes[node_num]
        model = node[model]

        self.nodes[node_num]['samples'] = self.sample_n_obsv_objects(n, model, run_fk=run_fk)
        return 0

    def rank_waypoint_samples(self, node_id):
        """
        re arrange all sampled points based on Prob Density
        """

        model = self.nodes[node_num]["model"]
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

    def rank_waypoint_free_samples(self, node_id):
        """
        re arrange all sampled points based on Prob Density
        """
        # TODO add desnity values to samples?

        model = self.nodes[node_id]["model"]
        samples = self.nodes[node_id]['free_samples']

        np_array = []
        for sample in samples:
            np_array.append(np.array(self.vectorizor(sample)))

        # pdb.set_trace()
        scores = model.score_samples(np_array)
        order = np.argsort(-scores)
        scores = scores[order]
        samples = np.asarray(samples)
        self.nodes[node_num]['free_samples'] = samples
        rospy.loginfo("keyframe %s samples have been reorderd", node_num)
        return 0


class KeyframeGraphDataGenerator():

    def generate_clusters(self, demonstrations):
        observations = []
        for demo in demonstrations:
            for obsv in demo.observations:
                observations.append(obsv)
        clusters = self.cluster_observations_by_id(observations)
        for cluster in clusters.values():
            self.assign_keyframe_type(cluster)
            self.assign_applied_constraints(cluster)
        return clusters

    def cluster_observations_by_id(self, observations):
        clusters = defaultdict(lambda: {"observations": []})

        for obsv in observations:
            keyframe_id = obsv.get_keyframe_info()[0]
            if keyframe_id is not None:
                clusters[keyframe_id]["observations"].append(obsv)

    def assign_keyframe_type(self, cluster):
        keyframe_type = cluster['observations'][0].get_keyframe_info()[1]
        cluster["keyframe_type"] = keyframe_type

    def assign_applied_constraints(self, cluster):
        applied_constraints = cluster['observations'][0].get_applied_constraint_data()
        cluster["applied_constraints"] = applied_constraints