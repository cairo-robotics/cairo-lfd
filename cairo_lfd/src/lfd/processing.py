"""
The processing.py module contains classes and methods for manipulating data/objects into difference forms.
"""
import numpy as np
from geometry_msgs.msg import Pose
from environment import Observation
from scipy.spatial.distance import euclidean
import pudb


def convert_data_to_pose(position, orientation):
    """
    Converts raw position and orientation data to a ROS message Pose object.

    Parameters
    ----------
    position : list
        List of numerical values representing x,y,z position.

    orientation : list
        List of numerical values representing the x,y,z,w values of a quaternion.

    normalize_quaternion : bool
        Flag indicating whether to normalize the values of the quaternion entries.

    Returns
    -------
    pose: geometry_msgs.msgs.Pose
        The Pose object
    """
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose


class SawyerSampleConverter(object):
    """
    Converts raw samples generated from models into Observation objects.

    Attributes
    ----------
    interface : object
        SawyerMoveitInterface to help run forward kinematics.
    """
    def __init__(self, interface):
        """
        Parameters
        ----------
        interface : object
            SawyerMoveitInterface to help run forward kinematics.
        """
        self.interface = interface

    def convert(self, sample, run_fk=False, normalize_quaternion=False):
        """
        Converts raw samples generated from models into Observation objects.

        Parameters
        ----------
        sample : list
            Raw sample to convert. Either joint configuration or pose [x,y,z,x,y,z,w] as a list.

        run_fk : bool
            Flag indicating whether to run forward kinematics or not.

        normalize_quaternion : bool
            Flag indicating whether to normalize the values of the quaternion entries.

        Returns
        -------
        obsv : lfd.environment.Observation
            Observation object constructed from the converted sample.
        """
        if run_fk is True:
            sample = self._run_foward_kinematics(sample)
        if normalize_quaternion:
            # Normalize the quaternion values otherwise they will not be valid. This may be needed if FK is done
            # using MoveIt's FK server. However, generally this is not needed if using Intera.
            sample[3], sample[4], sample[5], sample[6] = self._normalize_quaternion(sample[3], sample[4], sample[5], sample[6])

        if len(sample) > 7:
            # If length > 7, we know there must be joint data, so creat Obs w/ joints.
            obsv = Observation.init_samples(sample[0:3], sample[3:7], sample[7:14])
        else:
            obsv = Observation.init_samples(sample[0:3], sample[3:7], None)
        return obsv

    def _run_foward_kinematics(self, sample):
        """
        Runs forward kinematics on raw sample vector of robot joint configuration.

        Parameters
        ----------
        sample : list
            Raw sample joint configuration on which to run FK.

        Returns
        -------
        sample : list
            Appended list of numerical values now containing pose information.
        """
        pose = self.interface.get_FK_pose(sample)
        if pose is not None:
            sample = np.insert(sample, 0, pose.orientation.w, axis=0)
            sample = np.insert(sample, 0, pose.orientation.z, axis=0)
            sample = np.insert(sample, 0, pose.orientation.y, axis=0)
            sample = np.insert(sample, 0, pose.orientation.x, axis=0)
            sample = np.insert(sample, 0, pose.position.z, axis=0)
            sample = np.insert(sample, 0, pose.position.y, axis=0)
            sample = np.insert(sample, 0, pose.position.x, axis=0)
        return sample

    def _normalize_quaternion(self, x, y, z, w):
        """
        Normalizes quaternion values.

        Parameters
        ----------
        x,y,z,w : float
            Quaternion values

        Returns
        -------
        x,y,z,w : float
            Normalized quaternion values
        """
        normalize = np.sqrt(x**2 + y**2 +
                            z**2 + w**2)
        x = x / normalize
        y = y / normalize
        z = z / normalize
        w = w / normalize
        return x, y, z, w


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
            raise ValueError("Centrality must be either 'left', 'right', or 'center'.")

        if idx > len(observations) - 1:
            raise IndexError("Index not valid for given length of observations sequence.")
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
                    obsvs = obsvs + observations[idx + 1:idx + int(spread / 2) + 1]
                return obsvs
        return []


class ObjectRelativeDataProcessor(EuclideanDistanceMixin, ListWindowMixin):
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

    def generate_relative_data(self, observations):
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
                    target_position = observation.get_item_data(target_id)["position"]
                    relative_distances[target_id] = self._euclidean(item_position, target_position)
            robot_position = observation.get_robot_data()["position"]
            relative_distances[self.robot_id] = self._euclidean(item_position, robot_position)
            item_data["relative_distance"] = relative_distances

        # relative to robots
        relative_distances = {}
        robot_data = observation.get_robot_data()
        robot_position = robot_data["position"]
        for item_id in self.item_ids:
            item_position = observation.get_item_data(item_id)["position"]
            relative_distances[item_id] = self._euclidean(robot_position, item_position)
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
                        distances.append(ob.get_item_data(item_id)["relative_distance"].get(target_id))
                        timestamps.append(ob.get_timestamp())
                    if len(observation_window) == 0:
                        relative_velocities[target_id] = 0
                    else:
                        relative_velocities[target_id] = self._average_discrete_velocity(distances, timestamps)
            item_data["relative_velocity"] = relative_velocities

        # Now relative to robot, by accessing robot data separately.
        relative_velocities = {}
        robot_data = curr_observation.get_robot_data()
        for item_id in self.item_ids:
            distances = []
            timestamps = []
            for prior_ob in observation_window:
                distances.append(prior_ob.get_robot_data()["relative_distance"].get(item_id))
                timestamps.append(prior_ob.get_timestamp())
            if len(observation_window) == 0:
                relative_velocities[item_id] = 0
            else:
                relative_velocities[item_id] = self._average_discrete_velocity(distances, timestamps)
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
                        distances.append(ob.get_item_data(item_id)["relative_velocity"].get(target_id))
                        timestamps.append(ob.get_timestamp())
                    if len(observation_window) == 0:
                        relative_accelerations[target_id] = 0
                    else:
                        relative_accelerations[target_id] = self._average_discrete_velocity(distances, timestamps)
            item_data["relative_acceleration"] = relative_accelerations

        # Now relative to robot, by accessing robot data separately.
        relative_accelerations = {}
        robot_data = curr_observation.get_robot_data()
        for item_id in self.item_ids:
            distances = []
            timestamps = []
            for ob in observation_window:
                distances.append(ob.get_robot_data()["relative_velocity"].get(item_id))
                timestamps.append(ob.get_timestamp())
            if len(observation_window) == 0:
                relative_accelerations[item_id] = 0
            else:
                relative_accelerations[item_id] = self._average_discrete_velocity(distances, timestamps)
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
            raise ValueError("Every distance entry must have a corresponding timestamp")
        velocity_sum = 0
        for idx in range(0, len(distances)):
            if idx != len(distances) - 1:
                update = self._discrete_velocity(distances[idx], distances[idx + 1], timestamps[idx], timestamps[idx + 1])
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


class ObjectContactProcessor(EuclideanDistanceMixin, ListWindowMixin):

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

    def generate_object_contact_data(self, observations, window_size=8):
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
            window = self._retrieve_window(observations, idx, 'left', window_size)
            self._evaluate_contact(obsv, window)

    def _evaluate_contact(self, curr_observation, observation_window):    
        for item_id in self.item_ids:
            in_contact = {}
            item_data = curr_observation.get_item_data(item_id)
            for target_id in self.item_ids:
                if item_id != target_id:
                    within_threshold = []
                    for observation in observation_window:
                        item_position = observation.get_item_data(item_id)["position"]
                        target_position = observation.get_item_data(target_id)["position"]
                        distance = self._euclidean(item_position, target_position)
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
                target_position = observation.get_item_data(target_id)["position"]
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