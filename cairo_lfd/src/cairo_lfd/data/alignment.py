"""
The alignment.py module contains a variety of methods and classes used to align
Demonstrations captured during the LfD process.
"""
import copy

import rospy
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean


class DemonstrationAlignment(object):

    """
    Demonstration aligning class to align demonstrations, ensuring uniform constraint transitions across 
    all demonstrations.

    Attributes
    ----------
    vectorize_func : func
        A function used to vectorize the dictionary data of a demonstrations observations into the state space used by the DTW algorithm.
    """

    def __init__(self, vectorize_func):
        """
        Parameters
        ----------
        vectorize_func : func
            A function used to vectorize the dictionary data of a demonstrations observations into the state space used by the DTW algorithm.
        """
        self.vectorize_func = vectorize_func

    def align(self, demonstrations):

        """
        Alignment is performed using the FastDTW algorithm. It first separates trajectories that are constraint
        annotated, and aligns those first. Secondly, arbitrarily uses one of those trajectories as a reference
        against which to align all the remaining trajectories captured during demonstrations.

        Returns
        -------
        : tuple
            Returns the demonstrations each having a new parameter called aligned_observations and the constraint transitions
        """
        demonstrations = copy.deepcopy(demonstrations)
        rospy.loginfo("Performing alignment via the FastDTW algorithm...")
        if not len(demonstrations) > 1:
            raise AlignmentException("Error! You are attempting to align ONLY ONE OR ZERO demonstrations.")

        for demo in demonstrations:
            demo.aligned_observations = self._deep_copy_observations(demo.observations)

        constrained_demonstrations = [demo for demo in demonstrations if any([len(ob.data["applied_constraints"]) != 0 for ob in demo.observations])]

        # Align constrained demonstrations first, else if there are none, align all the trajectories without considering
        # constraints.
        if len(constrained_demonstrations) > 0:
            # Use the shortest demonstration as the reference. For now, this is an arbitrary alignment. 
            constrained_demonstrations.sort(key = lambda d: len(d.observations))
            reference_demo = constrained_demonstrations[0]

            # Align constrained demonstrations first to ensure equivalent constraint transition mappings.
            while self._check_for_equivalent_constraint_transitions(constrained_demonstrations) is False:
                for curr_demo in constrained_demonstrations:
                    # first loop collects applied constraints into shortest demonstration as master reference
                    alignments = self._get_alignment(curr_demo, reference_demo)
                    curr_demo.aligned_observations = alignments["current"]
                    reference_demo.aligned_observations = alignments["reference"]

            # Realign until uniform constraint transition mappings across all demonstrations
            while self._check_for_equivalent_constraint_transitions(demonstrations) is False:
                demonstrations.sort(key = lambda d: len(d.observations))
                for curr_demo in demonstrations:
                    alignments = self._get_alignment(curr_demo, reference_demo)
                    curr_demo.aligned_observations = alignments["current"]
                    reference_demo.aligned_observations = alignments["reference"]
        else:
            demonstrations.sort(key=lambda d: len(d.aligned_observations), reverse=True)
            reference_demo = demonstrations[0]
            for curr_demo in demonstrations:
                alignments = self._get_alignment(curr_demo, reference_demo)
                curr_demo.aligned_observations = alignments["current"]
                reference_demo.aligned_observations = alignments["reference"]

        constraint_transitions = self._get_universal_constraint_transitions(demonstrations)
        return (demonstrations, constraint_transitions)

    def _get_alignment(self, current_demo, reference_demo):

        """
        This function aligns two demonstrations and builds new observation lists.

        Alignment is performed using the FastDTW algorithm.

        Parameters
        ----------
        current_demo : Demonstration
           The current demonstration being aligned.

        current_demo : Demonstration
           The reference demonstration.

        Returns
        -------
        : dict
            Key: current; Value: A list of the current demonstration's new aligned observation list.
            Key: reference; Value: A list of the reference demonstration's new aligned observation list.
        """
        demos = [current_demo, reference_demo]
        demo_vectors = [self.vectorize_func(demo) for demo in demos]
        dist, idx_pairs = fastdtw(demo_vectors[0], demo_vectors[1], dist=euclidean)
        # idx_pairs = zip(path[0].tolist(), path[1].tolist())

        current_aligned_observations = []
        reference_aligned_observations = []
        for pair in idx_pairs:
            # build new observation trajectory
            current_ob = demos[0].get_observation_by_index(pair[0])
            reference_ob = demos[1].get_observation_by_index(pair[1])
            constraint_union = list(set(current_ob.data["applied_constraints"] + reference_ob.data["applied_constraints"]))
            current_ob.data["applied_constraints"] = constraint_union
            reference_ob.data["applied_constraints"] = constraint_union
            current_aligned_observations.append(current_ob)
            reference_aligned_observations.append(reference_ob)
        return {
            "current": current_aligned_observations,
            "reference": reference_aligned_observations
        }

    def _deep_copy_observations(self, observations):

        """
        Iterates over a list of observations and deep copies each.

        Parameters
        ----------
        observations : list
           Observations to be deep copied.

        Returns
        -------
        new_observations: list
            The deep copied observation list.
        """
        new_observations = []
        for ob in observations:
            new_observations.append(copy.deepcopy(ob))
        return new_observations

    def _get_applied_constraint_order(self, observations):

        """
        Returns the applied constraint order of a Demonstration's aligned observation list.

        Returns
        -------
        constraint_order: list
            List of list where each element is ordered by the sequence of the applied constraints and represents
            the set of constraints applied.
        """
        constraint_order = []
        curr = []
        for ob in observations:
            if curr != ob.data["applied_constraints"]:
                constraint_order.append(ob.data["applied_constraints"])
                curr = ob.data["applied_constraints"]
        return constraint_order

    def _check_for_equivalent_constraint_transitions(self, demonstrations):
        """
        Checks for equivalent constraint transitions across all demonstrations. This should
        occur after alignment.

        Parameters
        ----------
        demonstrations : list
           Demonstrations with which to generate the universal constraint transition map.

        Returns
        -------
        : boolean
            Boolean value with True indicating equivalent constraint transitions
        """
        mappings = [self._get_applied_constraint_order(demo.observations) for demo in demonstrations]

        if mappings[1:] == mappings[:-1]:
            return True
        else:
            return False

    def _get_universal_constraint_transitions(self, demonstrations):
        """
        Generates the universal constraint transition mapping for all ALIGNED demonstrations.
        Raises an exception if any of the demonstrations has a difference mapping than
        the others or if observations do not have aligned_observations instance variable populated.

        Parameters
        ----------
        demonstrations : list
           Demonstrations with which to generate the universal constraint transition map.

        Returns
        -------
        mapping: list
            The universal mapping of constraint transitions for all the demonstrations.
        """
        try:
            mappings = [self._get_applied_constraint_order(demo.aligned_observations) for demo in demonstrations]
        except AttributeError as e:
            raise AlignmentException("_get_universal_constraint_transitions() can only align demos\
                                     that have their aligned_observations instance variable populated")
        if mappings[1:] == mappings[:-1]:
            return mappings[0]
        else:
            raise Exception("Unequal constraint transition mappings!")


class AlignmentException(Exception):
    """
    Base class for exceptions in this module.
    """
    pass