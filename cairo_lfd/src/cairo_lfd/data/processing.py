"""
The processing.py module contains classes and methods for augmented data to create new data. For example,
relative distance and velocity data is calculated via the distances of objects.
"""
import copy

import rospy
import numpy as np
from scipy.spatial.distance import euclidean

from environment import Observation


class ProcessorPipeline():

    def __init__(self, processor_list):
        self.processors = processor_list

    def process(self, demonstrations):
        for idx, demo in enumerate(demonstrations):
            for processor in self.processors:
                processor.process(demo.observations)


class EuclideanDistanceMixin():

    def _euclidean(self, obj1_posistion, obj2_position):
        """
        Calculates the euclidean distance using SciPy's euclidean function

        Parameters
        ----------
        obj1_posistion : array-like
            List of x,y,z coordinates of object 1's position.

        obj2_position : array-like
            List of x,y,z coordinates of object 2's position.

        Returns
        -------
        : float
            The distance.
        """
        if isinstance(obj1_posistion, (list, np.ndarray)) and isinstance(obj2_position, (list, np.ndarray)):
            return euclidean(obj1_posistion, obj2_position)
        else:
            raise ValueError("Argument must be array-like (list or ndarray)")


class ListWindowMixin():

    def _retrieve_window(self, observations, idx, centrality='left', window_size=4):
        """
        Retrieves a number of observations before given index. The number shrinks if the index would make number of elements reach out of the sequence index bounds.

        Parameters
        ----------
        observations : list
            List of Observation objects.

        idx : int
            The index of the current observation.

        centrality : str
            Determines relative positioning of window relative to idx. One of three choices: 'left', 'right', 'center'.

        window_size : int
            Numeric size of window.

        Returns
        -------
        : list
            List of prior Observation objects.

        Note
        ----
        Along index boundary the window self adjusts to avoid index errors. This can result in fewer
        Observations than the given window size.
        """
        if centrality not in ['left', 'right', 'center']:
            raise ValueError(
                "Centrality must be either 'left', 'right', or 'center'.")

        if idx > len(observations) - 1:
            raise IndexError(
                "Index not valid for given length of observations sequence.")
        for spread in reversed(range(window_size + 1)):
            if centrality == 'left':
                if 0 <= idx - spread < len(observations) and 0 <= idx + spread < len(observations):
                    return observations[idx - spread:idx + 1]
            elif centrality == 'right':
                if 0 <= idx + spread < len(observations):
                    return observations[idx:idx + spread + 1]
            else:
                obsvs = observations[idx:idx + 1]
                if 0 <= idx - int(spread / 2) < len(observations):
                    obsvs = observations[idx - int(spread / 2):idx] + obsvs
                if 0 <= idx + int(spread / 2) < len(observations):
                    obsvs = obsvs + \
                        observations[idx + 1:idx + int(spread / 2) + 1]
                return obsvs
        return []


class RelativePositionProcessor(object):

    def __init__(self, item_ids, robot_id):
        """
        Parameters
        ----------
        item_ids : list
            List of environment 'item' ids.
        robot_id : int
            Id of robot in environment.
        """
        self.item_ids = item_ids
        self.robot_id = robot_id

    def process(self, observations):
        """

        Parameters
        ----------
        observations : list
            List of Observation objects.
        """
        for idx, obsv in enumerate(observations):
            self.generate_robot_relative_pos(obsv)
            self.generate_item_relative_pos(obsv)

    def generate_robot_relative_pos(self, obsv):
        # relative to robots
        for item_id in self.item_ids:
            relative_positions = {}
            item_data = obsv.get_item_data(item_id)
            item_position = item_data["position"]
            for target_id in self.item_ids:
                if item_id != target_id:
                    target_position = obsv.get_item_data(target_id)[
                        "position"]
                    relative_positions[target_id] = self._calculate_relative_position(
                        item_position, target_position)
            robot_position = obsv.get_robot_data()["position"]
            relative_positions[self.robot_id] = self._calculate_relative_position(
                item_position, robot_position)
            item_data["relative_positions"] = relative_positions

    def generate_item_relative_pos(self, obsv):
        # relative to robots
        relative_positions = {}
        robot_data = obsv.get_robot_data()
        robot_position = robot_data["position"]
        for item_id in self.item_ids:
            item_position = obsv.get_item_data(item_id)["position"]
            relative_positions[item_id] = self._calculate_relative_position(robot_position, item_position)
        robot_data["relative_positions"] = relative_positions

    def _calculate_relative_position(self, reference, target):
        x = target[0] - reference[0]
        y = target[1] - reference[1]
        z = target[2] - reference[2]
        return [x, y, z]


class RelativeKinematicsProcessor(EuclideanDistanceMixin, ListWindowMixin):
    """
    Calculates object to object relative distance, velocity and acceleration based on data
    contained in lists of Observations.

    Attributes
    ----------
    item_ids : list
        List of environment 'item' ids.
    """

    def __init__(self, item_ids, robot_id):
        """
        Parameters
        ----------
        item_ids : list
            List of environment 'item' ids.
        robot_id : int
            Id of robot in environment.
        """
        self.item_ids = item_ids
        self.robot_id = robot_id

    def process(self, observations):
        """
        Calculates relative distance, velocity and acceleration between item-item pairs 
        and item-robot pair. This is performed in-place: the dictionary data within each
        Observation acquires new key-value data.

        A window is chosen to provide averaged smoothing around a given observation to 
        avoid spiking velocities and accelerations due to noise. 

        Parameters
        ----------
        observations : list
            List of Observation objects.
        """
        for idx, obsv in enumerate(observations):
            self.relative_distance(obsv)
        for idx, obsv in enumerate(observations):
            window = self._retrieve_window(observations, idx, 'center', 4)
            self.relative_velocity(obsv, window)
        for idx, obsv in enumerate(observations):
            window = self._retrieve_window(observations, idx, 'center', 4)
            self.relative_acceleration(obsv, window)

    def relative_distance(self, observation):
        """
        Calculates relative distance, between item-item pairs and item-robot pair.

        This is performed in-place: the dictionary data within each Observation acquires new
        key-value relative distance.

        Parameters
        ----------
        observation : Observation
            observation object that acquires relative distance data
        """

        # between items and items & robot
        for item_id in self.item_ids:
            relative_distances = {}
            item_data = observation.get_item_data(item_id)
            item_position = item_data["position"]
            for target_id in self.item_ids:
                if item_id != target_id:
                    target_position = observation.get_item_data(target_id)[
                        "position"]
                    relative_distances[target_id] = self._euclidean(
                        item_position, target_position)
            robot_position = observation.get_robot_data()["position"]
            relative_distances[self.robot_id] = self._euclidean(
                item_position, robot_position)
            item_data["relative_distance"] = relative_distances

        # relative to robots
        relative_distances = {}
        robot_data = observation.get_robot_data()
        robot_position = robot_data["position"]
        for item_id in self.item_ids:
            item_position = observation.get_item_data(item_id)["position"]
            relative_distances[item_id] = self._euclidean(
                robot_position, item_position)
        robot_data["relative_distance"] = relative_distances

    def relative_velocity(self, curr_observation, observation_window):
        """
        Calculates relative velocity, between item-item pairs and item-robot pair.

        This is performed in-place: the dictionary data within each Observation acquires new
        key-value relative distance. This is performed by a simple discrete method that
        calculates the change in distance over the change in time of consecutive Observations.

        Parameters
        ----------
        curr_observation : Observation
            Current observation for which to calculate relative velocity.

        observation_window : list
            List of Observations used to calculate relative velocity as an average for the 
            current Observation.

        Notes
        -----
        Since Observations are sampled at fairly high rates, there can be substantial noise
        between any two observations that might spike a velocity calculation result. As a remedy,
        the resulting relative velocity is an average of the surrounding velocities calculated between
        consecutive Observations within a chosen window.
        """

        # between items and items & robot
        for item_id in self.item_ids:
            relative_velocities = {}
            item_data = curr_observation.get_item_data(item_id)
            # Loop through other items not the current item and calculate relative velocity:
            for target_id in self.item_ids + [self.robot_id]:
                if item_id != target_id:
                    distances = []
                    timestamps = []
                    for ob in observation_window:
                        distances.append(ob.get_item_data(item_id)[
                                         "relative_distance"].get(target_id))
                        timestamps.append(ob.get_timestamp())
                    if len(observation_window) == 0:
                        relative_velocities[target_id] = 0
                    else:
                        relative_velocities[target_id] = self._average_discrete_velocity(
                            distances, timestamps)
            item_data["relative_velocity"] = relative_velocities

        # Now relative to robot, by accessing robot data separately.
        relative_velocities = {}
        robot_data = curr_observation.get_robot_data()
        for item_id in self.item_ids:
            distances = []
            timestamps = []
            for prior_ob in observation_window:
                distances.append(prior_ob.get_robot_data()[
                                 "relative_distance"].get(item_id))
                timestamps.append(prior_ob.get_timestamp())
            if len(observation_window) == 0:
                relative_velocities[item_id] = 0
            else:
                relative_velocities[item_id] = self._average_discrete_velocity(
                    distances, timestamps)
        robot_data["relative_velocity"] = relative_velocities

    def relative_acceleration(self, curr_observation, observation_window):
        """
        Calculates relative velocity, between item-item pairs and item-robot pair.

        This is performed in-place: the dictionary data within each Observation acquires new
        key-value relative distance. This is performed by a simple discrete method that
        calculates the change in distance over the change in time of consecutive Observations.

        Parameters
        ----------
        curr_observation : Observation
            Current observation for which to calculate relative velocity.

        observation_window : list
            List of Observations used to calculate relative velocity as an average for the 
            current Observation.

        Notes
        -----
        Since Observations are sampled at fairly high rates, there can be substantial noise
        between any two observations that might spike a velocity calculation result. As a remedy,
        the resulting relative velocity is an average of the surrounding velocities calculated between
        consecutive Observations within a chosen window.
        """

        # between items and items & robot
        for item_id in self.item_ids:
            relative_accelerations = {}
            item_data = curr_observation.get_item_data(item_id)
            # Loop through other items not the current item and calculate relative velocity:
            for target_id in self.item_ids + [self.robot_id]:
                if item_id != target_id:
                    distances = []
                    timestamps = []
                    for ob in observation_window:
                        distances.append(ob.get_item_data(item_id)[
                                         "relative_velocity"].get(target_id))
                        timestamps.append(ob.get_timestamp())
                    if len(observation_window) == 0:
                        relative_accelerations[target_id] = 0
                    else:
                        relative_accelerations[target_id] = self._average_discrete_velocity(
                            distances, timestamps)
            item_data["relative_acceleration"] = relative_accelerations

        # Now relative to robot, by accessing robot data separately.
        relative_accelerations = {}
        robot_data = curr_observation.get_robot_data()
        for item_id in self.item_ids:
            distances = []
            timestamps = []
            for ob in observation_window:
                distances.append(ob.get_robot_data()[
                                 "relative_velocity"].get(item_id))
                timestamps.append(ob.get_timestamp())
            if len(observation_window) == 0:
                relative_accelerations[item_id] = 0
            else:
                relative_accelerations[item_id] = self._average_discrete_velocity(
                    distances, timestamps)
        robot_data["relative_acceleration"] = relative_accelerations

    def _average_discrete_velocity(self, distances, timestamps):
        """
        Calculates the discrete velocity between a number of points and returns the average
        discrete velocity.

        Each distance will have a corresponding timestamp.

        Parameters
        ----------
        distances : list
            List of distances.

        timestamps : list
            List of timestamps

        Returns
        -------
        : float
            Average discrete velocity..
        """
        if len(distances) != len(timestamps):
            raise ValueError(
                "Every distance entry must have a corresponding timestamp")
        velocity_sum = 0
        for idx in range(0, len(distances)):
            if idx != len(distances) - 1:
                update = self._discrete_velocity(
                    distances[idx], distances[idx + 1], timestamps[idx], timestamps[idx + 1])
                if abs(update) < 2.0:
                    velocity_sum += update
        return velocity_sum / int(len(distances) / 2)

    def _discrete_velocity(self, dist_before, dist_after, start_time, end_time):
        """
        Calculates the discrete velocity.

        Uses standard formula: change in distance over change in time.

        Parameters
        ----------
        dist_before : float
            The distance before the time step.

        dist_after : float
            The distance after the time step.

        start_time : float
            The start time,

        end_time : float
            The end time.

        Returns
        -------
        : float
            Discrete velocity
        """
        return (dist_after - dist_before) / abs(start_time - end_time)


class InContactProcessor(EuclideanDistanceMixin, ListWindowMixin):

    def __init__(self, item_ids, robot_id, threshold_distance=.06, window_percentage=.5):
        """
        Parameters
        ----------
        item_ids : list
            List of environment 'item' ids.
        robot_id : int
            Id of robot in environment.
        threshold_distance : float
            Distance within which two objects are considered in contact
        window_percentage : float
            Percentage of a window of observations that for which objects must be in contact for the current observation
            to be considered in contact.
        """
        self.item_ids = item_ids
        self.robot_id = robot_id
        self.threshold_distance = threshold_distance
        self.window_percentage = window_percentage

    def process(self, observations, window_size=8):
        """
        Calculates relative distance, velocity and acceleration between item-item pairs
        and item-robot pair. This is performed in-place: the dictionary data within each
        Observation acquires new key-value data.

        A window is chosen to provide averaged smoothing around a given observation to
        avoid spiking velocities and accelerations due to noise.

        Parameters
        ----------
        observations : list
           List of Observation objects.

        window_size : int
            The surrounding preceding window of observations used for evaluating contact.
        """
        for idx, obsv in enumerate(observations):
            window = self._retrieve_window(
                observations, idx, 'left', window_size)
            self._evaluate_contact(obsv, window)

    def _evaluate_contact(self, curr_observation, observation_window):
        for item_id in self.item_ids:
            in_contact = {}
            item_data = curr_observation.get_item_data(item_id)
            for target_id in self.item_ids:
                if item_id != target_id:
                    within_threshold = []
                    for observation in observation_window:
                        item_position = observation.get_item_data(item_id)[
                            "position"]
                        target_position = observation.get_item_data(target_id)[
                            "position"]
                        distance = self._euclidean(
                            item_position, target_position)
                        if distance <= self.threshold_distance:
                            within_threshold.append(True)
                        else:
                            within_threshold.append(False)
                    if within_threshold.count(True) / len(observation_window) >= self.window_percentage:
                        in_contact[target_id] = True
                    else:
                        in_contact[target_id] = False
            within_threshold = []
            for observation in observation_window:
                item_position = observation.get_item_data(item_id)["position"]
                robot_position = observation.get_robot_data()["position"]
                distance = self._euclidean(item_position, robot_position)
                if distance <= self.threshold_distance:
                    within_threshold.append(True)
                else:
                    within_threshold.append(False)
            if within_threshold.count(True) / len(observation_window) >= self.window_percentage:
                in_contact[self.robot_id] = True
            else:
                in_contact[self.robot_id] = False
            item_data["in_contact"] = in_contact

        # relative to robots
        in_contact = {}
        robot_data = curr_observation.get_robot_data()
        for target_id in self.item_ids:
            within_threshold = []
            for observation in observation_window:
                robot_position = observation.get_robot_data()["position"]
                target_position = observation.get_item_data(target_id)[
                    "position"]
                distance = self._euclidean(robot_position, target_position)
                if distance <= self.threshold_distance:
                    within_threshold.append(True)
                else:
                    within_threshold.append(False)
            if within_threshold.count(True) / len(observation_window) >= self.window_percentage:
                in_contact[target_id] = True
            else:
                in_contact[target_id] = False
        robot_data["in_contact"] = in_contact


class SphereOfInfluenceProcessor(EuclideanDistanceMixin):

    def __init__(self, item_ids, robot_id, threshold_distance=.1):
        self.item_ids = item_ids
        self.robot_id = robot_id
        self.threshold_distance = threshold_distance

    def process(self, observations):
        """
        Determines whether the end effector of a robotic arm is within the "sphere of influence" of
        items in the environment.  This is performed in-place: the dictionary data within each
        Observation acquires new key-value data.

        Parameters
        ----------
        observations : list
           List of Observation objects.
        """
        for obsv in observations:
            self._evaluate_SOI(obsv)

    def _evaluate_SOI(self, curr_observation):
        end_effector_position = curr_observation.get_robot_data()["position"]
        for item_id in self.item_ids:
            in_SOI = []
            item_position = curr_observation.get_item_data(item_id)[
                "position"]
            distance = self._euclidean(
                end_effector_position, item_position)
            if distance <= self.threshold_distance:
                in_SOI.append(item_id)
        curr_observation.data['robot']['in_SOI'] = in_SOI


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
            groupings = self._get_observation_groups(
                demo.aligned_observations, self.constraint_transitions)
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
                    labeled_group = self._set_applied_constraints_for_transition(
                        labeled_group)
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
        group_index_splits = [list(n) for n in np.array_split(
            list(range(0, len(group))), num_keyframes)]

        for group_idxs in group_index_splits:
            current_id = current_id + 1
            middle = (len(group_idxs)) / 2
            keyframe_idxs = self._retrieve_data_window(
                group_idxs, middle, window_size)
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
            groupings.append(self._get_observation_groups(
                demo.aligned_observations, self.constraint_transitions))
        combined_groups = list(zip(*groupings))
        for combination in combined_groups:
            average_length = int(
                sum([len(group) for group in combination]) / len(combination))
            keyframe_count = int(average_length / divisor)
            keyframe_counts.append(
                keyframe_count if keyframe_count != 0 else 1)
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
                curr_group.extend(self._retrieve_data_window(
                    observations, idx, window_size=6))
                curr_constraints = constraints.pop(
                    0) if len(constraints) > 0 else []
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

