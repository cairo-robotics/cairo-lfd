from dtw import fastdtw
from scipy.spatial.distance import euclidean
import copy


def vectorize_demonstration(demonstration):

    """
    Vectorizes a demonstration's observations through the union of the
    robot's postion and robot's joints.

    Parameters
    ----------
    demonstration : Demonstration
      Demonstraions to vectorize.

    Returns
    -------
    vectors : list
        List of observation vectors.
    """

    vectors = []
    for observation in demonstration.observations:
        position_data = observation.data["robot"]["position"]
        joint_data = observation.data["robot"]["joints"]
        vector = position_data + joint_data
        vectors.append(vector)
    return vectors


class DemonstrationAligner(object):

    """
    Demonstration aligning class to align demonstrations, ensuring uniform constraint transitions across 
    all demosntrations.
    """

    def __init__(self, demonstrations, vectorize_func):

        """
        Parameters
        ----------
        demonstrations : list
           List of demonstraions to align.

        vectorize_func : func
            A function used to vectorize the dictionary data of a demonstrations observations.
        """

        self.demonstrations = demonstrations
        self.vectorize_func = vectorize_func

    def align(self):

        """
        This function executes three identical loops where each demonstration is iteratively aligned against a 
        chosen reference demonstration. The reference demonstration performs an aggregation role, where it collects 
        the applied constraints against the demosntrations to which it is repeatedly aligned. The loop is repeated 
        as it forces a convergence onto a uniform constraint mapping.

        Alignment is performed using the FastDTW algorithm.

        Before the demonstrations are returned, the function iterates through each demonstration's aligned 
        observations to perform a deepcopy of each observation. This ensures that during keyframe labeling, 
        multiple references do not conflict as in place mutation occurs on observation objects during labeling.

        Returns
        -------
        self.demonstrations : tuple
            Returns the demonstrations each having a new parameter called aligned_observations.
        """

        if not len(self.demonstrations) > 1:
            raise Exception("Error! You are attempting to align ONLY ONE OR ZERO demonstrations.")
        self.demonstrations.sort(key = lambda d: len(d.observations))
        reference_demo = self.demonstrations[0]
        for curr_demo in self.demonstrations:
            # first loop collects applied constraints into shortest demonstration as master reference
            alignments = self._get_alignment(curr_demo, reference_demo)
            curr_demo.aligned_observations = alignments["current"]
            reference_demo.aligned_observations = alignments["reference"]
        for curr_demo in self.demonstrations:
            alignments = self._get_alignment(curr_demo, reference_demo)
            curr_demo.aligned_observations = alignments["current"]
            reference_demo.aligned_observations = alignments["reference"]
        for curr_demo in self.demonstrations:
            # By third loop, constraints have converged to an equivalent mapping.
            # Intuitively it makes some sense as iteratively running DTW will
            # converge onto some global alignment if a reference vector is always used.
            alignments = self._get_alignment(curr_demo, reference_demo)
            curr_demo.aligned_observations = alignments["current"]
            reference_demo.aligned_observations = alignments["reference"]
        for demo in self.demonstrations:
            demo.aligned_observations = self._deep_copy_observations(demo.aligned_observations)
        constraint_transitions = self._get_universal_constraint_transitions(self.demonstrations)
        return (self.demonstrations, constraint_transitions)

    def _get_alignment(self, current_demo, reference_demo):

        """
        This function aligns two demonstrations and builds new observation lists.

        Alignment is performed using the FastDTW algorithm.

        Parameters
        ----------
        current_demo : Demonstration
           The current demosntration being aligned.

        current_demo : Demonstration
           The reference demosntration.

        Returns
        -------
        : dict
            Key: current; Value: A list of the current demonstration's new aligned observation list.
            Key: reference; Value: A list of the reference demonstration's new aligned observation list.
        """

        demos = [current_demo, reference_demo]
        demo_vectors = [self.vectorize_func(demo) for demo in demos]
        dist, cost, acc, path = fastdtw(demo_vectors[0], demo_vectors[1], dist=euclidean)
        idx_pairs = zip(path[0].tolist(), path[1].tolist())

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
        Iterates of a list of observations and deep copies each.

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

    def _get_universal_constraint_transitions(self, demonstrations):

        """
        Generates the universal constraint transition mapping for all demosntraionts.
        Raises an exception if any of the demosntrations has a difference mapping than
        the others.

        Parameters
        ----------
        demonstrations : list
           Demosntrations with which to generate the universal constraint transition map.

        Returns
        -------
        mapping: list
            The universal mapping of constraint transitions for all the demonstraionts.
        """

        mappings = [demo.get_applied_constraint_order() for demo in demonstrations]
        if mappings[1:] == mappings[:-1]:
            return mappings[0]
        else:
            raise Exception("Unequivalent constraint transition mappings!")
