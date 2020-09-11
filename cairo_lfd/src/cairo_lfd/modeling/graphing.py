"""
The module graphing.py contains classes for building graphical structures of learned skills for Keyframe-based LfD.
"""
import itertools
from collections import defaultdict
import rospy
import numpy as np
from networkx import MultiDiGraph


class KeyframeGraph(MultiDiGraph):
    """
    NetworkX MultiDiGraph extended graph class for containing keyframes and their corresponding models.
    """
    def __init__(self):
        MultiDiGraph.__init__(self)

    def get_keyframe_sequence(self):
        """
        Provides the keyframe sequence in order.

        Returns
        -------
        node_chain : list
            List of keyframe/node sequence.
        """
        node_chain = list(set(itertools.chain(*self.edges())))
        node_chain.sort()
        return node_chain

    def cull_node(self, node):
        """
        Takes received node and removes it from the task graph

        Parameters
        ----------
        node: hashable networkx node name (usually int)
            the node to be removed from network x graph
        """
        next_nodes = [x for x in self.successors(node)]
        prev_nodes = [x for x in self.predecessors(node)]
        if next_nodes == []:
            prev_node = prev_nodes[0]
            self.remove_edge(prev_node, node)
            rospy.loginfo("Keyframe %s has been culled", node)
        elif prev_nodes == []:
            next_node = next_nodes[0]
            self.remove_edge(node, next_node)
            rospy.loginfo("Keyframe %s has been culled", node)
        else:
            prev_node = prev_nodes[0]
            next_node = next_nodes[0]
            self.add_edge(prev_node, next_node)
            self.remove_edge(prev_node, node)
            self.remove_edge(node, next_node)
            rospy.loginfo("Keyframe %s has been culled", node)

    def fit_models(self, observation_vectorizor):
        """
        Fits all models in graph. Expects that each keyframe node in graph has populated "observation" key
        as a list of Observation objects.

        Parameters
        ----------
        observation_vectorizor : function
            Function that takes observations stored in graph and converts their data into vector form for use by model
            fitting function.
        """
        for node in self.nodes():
            np_array = []
            for obsv in self.nodes[node]["observations"]:
                vector = np.array(observation_vectorizor(obsv))
                np_array.append(np.array(observation_vectorizor(obsv)))
            np_array = np.array(np_array)
            self.nodes[node]["model"].fit(np_array)

    def fit_models_on_valid_samples(self, node, observation_vectorizor):
        np_array = []
        for obsv in self.nodes[node]["samples"]:
            vector = np.array(observation_vectorizor(obsv))
            np_array.append(np.array(observation_vectorizor(obsv)))
        np_array = np.array(np_array)
        self.nodes[node]["model"].fit(np_array)

    def identify_primal_observations(self, observation_vectorizor):
        for node in self.nodes():
            np_array = []
            best_obs = None
            old_score = -np.inf
            for obsv in self.nodes[node]["observations"]:
                vector = np.array(observation_vectorizor(obsv))
                sample = np.array(observation_vectorizor(obsv))
                new_score = self.nodes[node]["model"].score_samples(np.array([sample]))[0]
                if new_score > old_score:
                    best_obs = obsv
                    old_score = new_score
            self.nodes[node]["primal_observation"] = best_obs


class KeyframeClustering():
    """
    Cluster observations by keyframe ID and gathers pertinent information regarding each keyframe. This will
    be used by a KeyframeGraph object to build nodes, each of which represent a keyframe.
    """
    def get_clusters(self, demonstrations):
        """
        Generates clustered Observations from a list of Demonstrations with labeled observations.

        Parameters
        ----------
        demonstrations : list
            List of the Demonstration objects from which to cluster labeled observations according to labeled id.

        Returns
        -------
        clusters : dict
            Dictionary of clusters with top level keys the keyframe IDs and values another dictionary populated
            with pertinent information associated with each keyframe (i.e. keyframe type, applied constraints etc,.)
        """
        observations = []
        for demo in demonstrations:
            for obsv in demo.labeled_observations:
                observations.append(obsv)
        clusters = self._cluster_observations_by_id(observations)
        for cluster in clusters.values():
            self._assign_keyframe_type(cluster)
            self._assign_applied_constraints(cluster)
        return clusters


    def _cluster_observations_by_id(self, observations):
        """
        Takes in a the entirety of observations from all Demonstrations and groups them together
        by keyframe ID. This

        Parameters
        ----------
        observations : list
            List of the Observation objects to cluster.

        Returns
        -------
        clusters : dict
            Dictionary of clusters with top level keys the keyframe IDs and values another dictionary populated
            with an 'observations' key and a list of Observations as the value.
        """
        clusters = defaultdict(lambda: {"observations": []})

        for obsv in observations:
            keyframe_id = obsv.get_keyframe_info()[0]
            if keyframe_id is not None:
                clusters[keyframe_id]["observations"].append(obsv)
        return clusters

    def _assign_keyframe_type(self, cluster):
        """
        Assigns the keyframe type for the given keyframe cluster.

        Parameters
        ----------
        cluster : dict
            Dictionary to assign the keyframe type.
        """
        keyframe_type = cluster['observations'][0].get_keyframe_info()[1]
        cluster["keyframe_type"] = keyframe_type

    def _assign_applied_constraints(self, cluster):
        """
        Assigns the applied constraints for the given keyframe cluster.

        Parameters
        ----------
        cluster : dict
            Dictionary to assign the applied constraints..
        """
        applied_constraints = cluster['observations'][0].get_applied_constraint_data()
        cluster["applied_constraints"] = applied_constraints


class IntermediateTrajectories():

    def get_trajectories(self, demonstrations):
        """
        Takes in a list of demonstrations on which to extract the groups of intermediate trajectores (slices from each demo) that represent intermediate trajectories leading up to a constraint transition region. 

        Parameters
        ----------
        demonstrations : list
            List of the Demonstration objects.

        Returns
        -------
        clusters : dict
            Dictionary of groups of trajectories. The key represents the keyframe_id at which the slices of the trajectories terminate.
        """
        id_sequence = _constraint_transition_id_sequence(demonstrations[0])
        trajectory_groups = {}
        for keyframe_id in id_sequence:
            group = []
            for demo in demonstrations:
                trajectory_slice = []
                for obsv in demo.labeled_observations:
                    keyframe_id, keyframe_type = obsv.get_keyframe_info()
                    if keyframe_id != keyframe_id:
                        trajectory_slice.append(obsv)
                    else:
                        break
                group.append(trajectory_slice)
            trajectory_groups[keyframe_id] = group
        return trajectory_groups

    def _constraint_transition_id_sequence(self, demonstration):
        sequence = []
        for obsv in demonstration.labeled_observations:
            keyframe_id, keyframe_type = obsv.get_keyframe_info()
            if keyframe_type == "constraint_transition":
                if keyframe_id not in sequence:
                    sequence.append(keyframe_id)
        return sequence

