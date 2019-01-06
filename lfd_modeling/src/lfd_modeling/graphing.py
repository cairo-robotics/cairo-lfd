import itertools
from collections import defaultdict
import rospy
import numpy as np
from networkx import MultiDiGraph


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

    def fit_models(self, observation_vectorizor, bandwidth=.001):
        """
        create models with observations in node
        """
        for node in self.nodes():
            np_array = []
            for obsv in self[node]["observations"]:
                np_array.append(np.array(observation_vectorizor(obsv)))
            self[node]["model"].fit(np_array)


class ObservationClusterer():

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