import rospy
import numpy as np
import copy


class ConstraintKeyframeLabeler():
    """
    Keyframe labeling class.

    This class depends on constraint aligned demonstrations. This means that all demonstrations should have the same
    sequence of constraint transitions. Without such alignment, the class functions will fail ungracefully.

    Attributes
    ----------
    demonstrations : list
       List of demonstrations. These must be constraint aligned

    constraint_transitions : list
        A 2D list containing the set of constraint transitions that are applicable to all of the aligned
        demonstrations.

    """
    def __init__(self, aligned_demonstrations, constraint_transitions):
        """
        Parameters
        ----------
        aligned_demonstrations : list
           List of demonstrations. These must be constraint aligned

        constraint_transitions : list
            A 2D list containing the set of constraint transitions that are applicable to all of the aligned
            demonstrations.
        """
        self.demonstrations = aligned_demonstrations
        self.constraint_transitions = constraint_transitions

    def label_demonstrations(self, divisor=20, keyframe_window_size=8):
        """
        This function serves to take each demonstration and create a list of observations labeled with keyframe_ids.
        For each demonstration, the function gets the observation grouping and then iteratively calls
        _get_labeled_group() from which it extends a list using the function's returned labeled_group. This list becomes
        the labeled_observations list of observation objects assigned to the demonstration object.

        Parameters
        ----------
        group_divisor : int
           The divisor used by the _get_keyframe_count_per_group function

        keyframe_window_size: string
             The size of the window of each split used to capture a subset of data for the keyframe.

        Returns
        -------
        demonstrations : tuple
            Returns the classes demonstrations attribute each of which will have a new parameter assigned with a list called
            'labeled_observations'.
        """
        rospy.loginfo("Labeling keyframe groups...")
        keyframe_counts = self._get_keyframe_count_per_group(divisor)
        for demo in self.demonstrations:
            groupings = self._get_observation_groups(demo.aligned_observations, self.constraint_transitions)
            labeled_observations = []
            current_id = 0
            for idx, group in enumerate(groupings):
                # Recall that every even index in groupings is a regular group while all odd indices are transition groups.
                if idx % 2 == 0:
                    keyframe_type = "regular"
                    current_id, labeled_group = self._get_labeled_group(group, keyframe_type, current_id,
                                                                        keyframe_counts[idx], keyframe_window_size)
                else:
                    keyframe_type = "constraint_transition"
                    current_id, labeled_group = self._get_labeled_group(group, keyframe_type, current_id,
                                                                        keyframe_counts[idx], 4)
                    labeled_group = self._set_applied_constraints_for_transition(labeled_group)
                labeled_observations.extend(labeled_group)
            demo.labeled_observations = labeled_observations
        return self.demonstrations

    def _get_labeled_group(self, observation_group, keyframe_type, current_id, num_keyframes, window_size):
        """
        This function takes in a group, generates a list of its indices, and splits those indices into n lists were
        n is the number of keyframes. Each of these list of indices represent the observations available to constitute
        a keyframe.

        Using the index splits, the center of each of those splits is calculated, and a window of elements is taken
        around that center. This window of indices will be the indices of the observation_group's elements that
        will be used for a keyframe.

        The observations are labeled with keyframe_ids by iterating over the index splits and labeling the data with an
        increasing current_id. This has the effect of shrinking the keyframes, purposefully under utilizing the
        demonstration's observations.

        Parameters
        ----------
        observation_group : int
           A list of observations to be separated into keyframes.

        keyframe_type: string
            Either 'regular' or 'constraint_transition'. Used to label observation's keyframe type.

        current_id: int
            The current starting index of the next set of keyframes to be made by this function.

        num_keyframes: int
            The number of keyframes that the observation_group list should be split into.

        window_size: int
            The size of the window of each split used to capture a subset of data for the keyframe.

        Returns
        -------
        (current_id, labeled_observations) : tuple
            Returns a tuple of the current_id (so it can be passed to the next call of this function) and a list of
            labeled observations.
        """
        labeled_observations = []
        group = copy.deepcopy(observation_group)
        group_index_splits = [list(n) for n in np.array_split(list(range(0, len(group))), num_keyframes)]

        for group_idxs in group_index_splits:
            current_id = current_id + 1
            middle = (len(group_idxs)) / 2
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

    def _set_applied_constraints_for_transition(self, constraint_transition_group):
        """
        This function takes a list observations associated with a constraint transition keyframe label
        and makes sure the ending applied constraints is set on all of the points in the group labeled
        as a constraint_transition keyframe_type.

        Parameters
        ----------
        constraint_transition_group : list
            The list of constraint transition observations

        Returns
        -------
        constraint_transition_group : list
        """
        last_idx = len(constraint_transition_group) - 1
        for ob in constraint_transition_group:
            if ob.data["keyframe_type"] == "constraint_transition":
                ob.data["applied_constraints"] = constraint_transition_group[last_idx].data["applied_constraints"]
        return constraint_transition_group

    def _get_keyframe_count_per_group(self, divisor=20):
        """
        This function calculates the number of keyframes for each group. A group is a precursor to keyframes. Generally,
        the order of groups is as follows:

             Group 1      Group 2   Group 3
        D1: r r r r       t t t t   r r r r r r r
        D2: r r r r r r   t t t t   r r r r r r

        Group 1, in the above example, consists of all the regular data needed perform regular keyframing prior to a transition
        region. The purpose of this function is to calculate how many keyframes each group ought to have.

        Parameters
        ----------
        divisor : int
            A divisor that divides the average length of a group of 'regular' observations into n number of keyframes
            for that group

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
            average_length = int(sum([len(group) for group in combination]) / len(combination))
            keyframe_count = int(average_length / divisor)
            keyframe_counts.append(keyframe_count if keyframe_count != 0 else 1)
        return keyframe_counts

    def _get_observation_groups(self, observations, constraint_transitions):
        """
        This function generates groups of observations based on the constraint transitions of a demonstration.

        It will create a list of observation lists, where each index alternates between being a group of regular
        observations and observations that surround a constraint transition. The group structure is generated
        using _generate_group_structure().

        Parameters
        ----------
        observations : list
           List of observations to group according to the constraint_transitions of the demonstration.

        constraint_transitions : list
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

        All even indices represent regular groups and odd indices represent transition groups.

        Parameters
        ----------

        constraint_transitions : list
            A 2D list containing the sets of constraint transitions.

        Returns
        -------
        groups : list
            2D list of the length of the number of groups.
        """
        groups = []
        for idx in range(2 * len(constraint_transitions) + 1):
            groups.append([])
        return groups

    def _retrieve_data_window(self, sequence, central_idx, window_size=10):
        """
        Retrieves a window of elements from a sequence. The window is centered by central_idx.
        The window size will shrink if the central index would make the window out of the sequence index bounds.

        Parameters
        ----------

        sequence : list
            Any iterable.

        central_idx : int
            The index to be the center of the window.

        window_size : int
            Size of window of elements to grab surrounding that center index from the sequence.

        Returns
        -------
        : list
            List of elements captured by the window.
        """
        for spread in reversed(range(int(window_size / 2) + 1)):
            if 0 <= central_idx - spread < len(sequence) and 0 <= central_idx + spread < len(sequence):
                return sequence[central_idx - spread:central_idx + spread + 1]
        return []


class ObjectContactSegmentation():

    def __init__(self):
        pass

    def assign_segments(observations):
        data[0]['segment'] = 1 
        segchange = False # track whether segment increment is necessary

        # Parse
        prev_set = set()
        for i in range(1, len(data)): # iterate through blocks
            for j in range(len(data[i]['items'])): # iterate through items
                for key in data[i]['items'][j]['in_contact']: # iterate through in_contact
                    if data[i]['items'][j]['in_contact'][key] != data[i - 1]['items'][j]['in_contact'][key]:
                        segchange = True
            for key in data[i]['robot']['in_contact']: # iterate through in_contact for robot
                if data[i]['robot']['in_contact'][key] != data[i - 1]['robot']['in_contact'][key]:
                    segchange = True
            if segchange is True:
                data[i]['segment'] = data[i - 1]['segment'] + 1
                segchange = False
            else:
                data[i]['segment'] = data[i - 1]['segment']