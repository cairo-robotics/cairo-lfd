from lfd_processor.environment import Demonstration, Observation
from lfd_processor.data_io import DataImporter, DataExporter
from lfd_processor.alignment import DemonstrationAligner, vectorize_demonstration
import numpy as np
import copy
import os


class ConstraintAnalyzer():
    """
    Constraint analysis class to evaluate observations for constraints.
    """
    def __init__(self, environment):
        """
        Parameters
        ----------
        aligned_demonstrations : list
           List of demonstraionts. These must be constraint aligned

        constraint_transitions : 2D list
            A 2D list containing the set of constraint transitions that are applicable to all of the aligned demosntrations.
        """
        self.environment = environment

    def applied_constraint_evaluator(self, observations):
        """
        This function evaluates observations for constraints that were triggered during the demonstration. It will label a demonstration's entire
        list of observations with the constraints that were triggered and wheter or not they are still applicable.

        New constraints are those where the triggered constraints are different from the previously applied constraints:
            triggered = observation.get_triggered_constraint_data()
            new = list(set(triggered)-set(prev))

        The evaluated constraints are those that are still valid from the previous observations applied constraints.
            evaluated = self._evaluator(constraint_ids=prev, observation=observation)

        The current observations applied constraints is the union of the evaluted constraints and new constraints
             applied = list(set(evaluated).union(set(new)))

        Parameters
        ----------
        observations : int
           List of observations to be evaluated for constraints.
        """
        prev = []
        for observation in observations:
            triggered = observation.get_triggered_constraint_data()
            new = list(set(triggered)-set(prev))
            evaluated = self._evaluator(constraint_ids=prev, observation=observation)
            applied = list(set(evaluated).union(set(new)))
            prev = applied
            observation.data["applied_constraints"] = applied

    def _evaluator(self, constraint_ids, observation):
        """
        This function evaluates an observation for all the constraints in the list constraint_ids. It depends on being able to access the constraint objects
        from the self.environment object. Every constraint object should have an evaluate function that takes in the environment and the observation.

        Parameters
        ----------
        constraint_ids : list
            List of constraint id's to evalaute.

        observation : Observation
            The observation to evaluate for the constraints.
    
        Returns
        -------
        valid_constraints : list
            Returns the list of valid cosntraints evaluated for the observation. 
        """
        if constraint_ids is not []:
            valid_constraints = []
            for constraint_id in constraint_ids:
                constraint = self.environment.get_constraint_by_id(constraint_id)
                result = constraint.evaluate(self.environment, observation)
                if result is 1:
                    valid_constraints.append(constraint_id)
            return valid_constraints
        else:
            return []


class DemonstrationKeyframeGrouper():
    """
    Keyframe grouping class. 

    This class depends on constraint aligned demosntrations. This means that all demosntrations should have the same sequence of constraint transitions.
    Without such alignment, the class functions will fail ungracefully.
    """

    def __init__(self, aligned_demonstrations, constraint_transitions):
        """
        Parameters
        ----------
        aligned_demonstrations : list
           List of demonstraionts. These must be constraint aligned

        constraint_transitions : 2D list
            A 2D list containing the set of constraint transitions that are applicable to all of the aligned demosntrations.
        """
        self.demonstrations = aligned_demonstrations
        self.constraint_transitions = constraint_transitions

    def group_data(self, divisor = 20, keyframe_window_size = 8):
        """
        This function serves to take each demonstration and create a list of observations labeled with keyframe_ids.
        For each demonstation, the function gets the observation grouping and then iteratively calls _get_labeled_group() from which
        it extends a list using returned labeled_group. This list becomes the labeled_observations list of observation obejcts assigned 
        to the demonstration object.

        Parameters
        ----------
        group_divisor : int
           The divisor used by the _get_keyframe_count_per_group function

        keyframe_window_size: string
             The size of the window of each split used to capture a subset of data for the keyframe.

        Returns
        -------
        self.demonstrations : tuple
            Returns the demonstrations each of which will have a new parameter assigned with a list called 'labeled_observations'.
        """
        keyframe_counts = self._get_keyframe_count_per_group(divisor)
        for demo in self.demonstrations:
            groupings = self._get_observation_groups(demo.aligned_observations, self.constraint_transitions)
            labeled_observations = []
            current_id = 0
            for idx, group in enumerate(groupings):
                # Recall that every even index in groupings is a regular group while all odd indices are transition groups.
                if idx%2 == 0:
                    keyframe_type = "regular"
                    current_id, labeled_group = self._get_labeled_group(group, keyframe_type, current_id, keyframe_counts[idx], keyframe_window_size)
                else:
                    keyframe_type = "constraint_transition"
                    current_id, labeled_group = self._get_labeled_group(group, keyframe_type, current_id, keyframe_counts[idx], 4)
                labeled_observations.extend(labeled_group)
            demo.labeled_observations = labeled_observations
        return self.demonstrations

    def _get_labeled_group(self, observation_group, keyframe_type, current_id, num_keyframes, window_size):
        """
        This function takes in a group, generates a list of its indices, and splits those indices into n lists were n is the number of keyframes. Each of these 
        list of indices represent the observations available to constitute a keyframe.
        
        Using the index splits, the center of each of those splits is calculated, and window of elements is taken around that center. This window of indices will 
        be the indices of the observation_group list that will be used for a keyframe.

        The observations are labeled with keyframe_ids by iterating over the index splits, caputring the windows, and labeling the data with an increasing current_id. This has the effect of shrinking the keyframes, purposefully under utilizing
        the demonstration's observations. 



        Parameters
        ----------
        observation_group : int
           A list of obervations to be separated into keyframes.

        keyframe_type: string
            Either 'regular' or 'constraint_transition'. Used to label observations.

        current_id: int
            The current starting index of the next set of keyframes to be made by this function.

        num_keyframes: int
            The number of keyframes that the observation_group list should be split into.

        window_size: int
            The size of the window of each split used to capture a subset of data for the keyframe.

        Returns
        -------
        (current_id, labeled_observations) : tuple
            Returns a tuple of the current_id (so it can be passed to the next call of this function) and a list of labeled_observations.
        """
        labeled_observations = []
        group = copy.deepcopy(observation_group)
        group_index_splits = [list(n) for n in np.array_split(list(range(0, len(group)-1)), num_keyframes)]

        for group_idxs in group_index_splits:
            current_id = current_id + 1
            middle = (len(group_idxs))/2
            keyframe_idxs = self._retrieve_data_window(group_idxs, middle, window_size)
            ignored_idxs = list(set(group_idxs) - set(keyframe_idxs))
            for i in keyframe_idxs:
                group[i].data["keyframe_id"] = current_id
                group[i].data["keyframe_type"] = keyframe_type
            for j in ignored_idxs:
                group[j].data["keyframe_id"] = None
                group[j].data["keyframe_type"] = None
            for idx in group_idxs:
                # to retain ordering, loop over group_idxs and append each observation in group after they've beel labeled.
                labeled_observations.append(group[idx])
        return (current_id, labeled_observations)

    def _get_keyframe_count_per_group(self, divisor = 20):
        """
        This function calculates the number of keyframes for each group. A group is a precursor to keyframes. Generally,
        the order of groups is as follows:

             Group 1         Group2
        D1: r r r r         t t t t
        D2: r r r r r r     t t t t

        Group 1, in the above example, consists of all the regular data needed peform regular keyframing. The purpose of this function
        is to calculate how many keyframes each group ought to have.

        Parameters
        ----------
        divisor : int
            A divisor that divides the average length of a group of 'regular' observations into n number of keyframes for that group

        Returns
        -------
        keyframe_counts : list
           A list of the number of keyframes per group.
        """
        keyframe_counts = []
        groupings = []
        for demo in self.demonstrations:
            groupings.append(self._get_observation_groups(demo.aligned_observations, self.constraint_transitions))
        combined_groups = list(zip(*groupings))
        for combination in combined_groups:
            average_length = int(sum([len(group) for group in combination])/len(combination))
            keyframe_count = int(average_length/divisor)
            keyframe_counts.append(keyframe_count if keyframe_count != 0 else 1)
        return keyframe_counts

    def _get_observation_groups(self, observations, constraint_transitions):
        """
        This function generates groups of observations based on the constraint transitions of a demonstration.

        It will create a list of observation lists, where each index alternates between being a group of regular observations
        and observations that surround a constraint transition. The group structure is generated using _generate_group_structure().

        Parameters
        ----------
        observations : list
           List of observations to group according to the constraint_transitions of the demonstration.

        constraint_transitions : 2D list
            A 2D list containing the sets of constraint transitions.

        Returns
        -------
        groups : list
           A list of groups with each entry containing a list observations for that group.
        """
        constraints = copy.deepcopy(constraint_transitions)
        groups = self._generate_group_structure(constraints)  
        curr_constraints = []
        counter = 0
        for idx, ob in enumerate(observations):
            curr_group = groups[counter]
            if ob.data["applied_constraints"] != curr_constraints:
                counter += 1
                curr_group = groups[counter]
                curr_group.extend(self._retrieve_data_window(observations, idx, window_size=6))
                curr_constraints = constraints.pop(0) if len(constraints) > 0 else []
                counter += 1
                curr_group = groups[counter]
            else:
                curr_group.append(ob)
        return groups

    def _generate_group_structure(self, constraint_transitions):
        """
        This function generates a 2D list of the length of the number of groups.

        Example:
        constraint_transitions = [[1],[1, 2],[]]

        results in:

        groups = [[], [], [], [], [], [], []]

        All even indices represet regular groups and odd indices represent transition groups.

        Parameters
        ----------

        constraint_transitions : 2D list
            A 2D list containing the sets of constraint transitions.

        Returns
        -------
        groups : list
            2D list of the length of the number of groups.
        """
        groups = []
        for idx in range(2*len(constraint_transitions)+1):
            groups.append([])
        return groups

    def _retrieve_data_window(self, sequence, central_idx, window_size=10):
        """
        Retrieves a window of elements from a sequence. The window is centered by central_idx.
        The window size will shrink if the central index would make the window out of the sequence index.

        Parameters
        ----------

        sequence : 2D list
            Any iterable.

        central_idx : int
            The index to be the center of the window.

        window_size : int
            Size of windown of elements to grab surrounding that center index from the sequence.

        Returns
        -------
        : list
            List of elements captured by the window.
        """
        for spread in reversed(range(int(window_size/2)+1)):
            if 0 <= central_idx-spread < len(sequence) and 0 <= central_idx+spread < len(sequence):
                return sequence[central_idx-spread:central_idx+spread+1]
        return []
