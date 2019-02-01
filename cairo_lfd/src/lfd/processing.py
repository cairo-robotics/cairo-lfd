"""
The processing.py module contains classes and methods for manipulating data/objects into difference forms.
"""
import numpy as np
from geometry_msgs.msg import Pose
from environment import Observation
from scipy.spatial.distance import euclidean
from itertools import combinations
from pudb import set_trace


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


class ObjectRelativeDataProcessor():

    def __init__(self, item_ids, robot_id):
        self.item_ids = item_ids
        self.robot_id = robot_id

    def generate_relative_data(self, observations):
        for idx, obsv in enumerate(observations):
            self.relative_distance(obsv)
            prior_observations = self._retrieve_prior_observations(observations, idx, 4)
            self.relative_velocity(obsv, prior_observations)

    def relative_distance(self, observation):
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

    def relative_velocity(self, curr_observation, prior_observations):
        # between items and items & robot
        curr_time = curr_observation.get_timestamp()
        for item_id in self.item_ids:
            relative_velocities = {}
            item_data = curr_observation.get_item_data(item_id)
            item_relative_distances = item_data["relative_distance"]
            # Loop through other items not the current item and calculate relative velocity:
            for target_id in self.item_ids + [self.robot_id]:
                if item_id != target_id:
                    distances = []
                    timestamps = []
                    curr_target_distance = item_relative_distances.get(target_id)
                    for prior_ob in prior_observations:
                        distances.append(prior_ob.get_item_data(item_id)["relative_distance"].get(target_id))
                        timestamps.append(prior_ob.get_timestamp())
                    distances.append(curr_target_distance)
                    timestamps.append(curr_time)
                    reversed(distances)
                    reversed(timestamps)
                    if len(prior_observations) == 0:
                        relative_velocities[target_id] = 0
                    else:
                        relative_velocities[target_id] = self._average_discrete_velocity(distances, timestamps)
            item_data["relative_velocity"] = relative_velocities

        # Now relative to robot, by accessing robot data separately.
        relative_velocities = {}
        robot_data = curr_observation.get_robot_data()
        item_relative_distances = robot_data["relative_distance"]
        for item_id in self.item_ids:
            distances = []
            timestamps = []
            curr_target_distance = item_relative_distances.get(item_id)
            for prior_ob in prior_observations:
                distances.append(prior_ob.get_robot_data()["relative_distance"].get(item_id))
                timestamps.append(prior_ob.get_timestamp())
            distances.append(curr_target_distance)
            timestamps.append(curr_time)
            reversed(distances)
            reversed(timestamps)
            if len(prior_observations) == 0:
                relative_velocities[item_id] = 0
            else:
                relative_velocities[item_id] = self._average_discrete_velocity(distances, timestamps)
        robot_data["relative_velocity"] = relative_velocities

    def relative_acceleration(self, obj1_window, obj2_window):
        pass

    def _retrieve_prior_observations(self, observations, idx, number=1):
        """
        Retrieves a number of observations before given index. The number shrinks if the index would make number of elements reach out of the sequence index bounds.

        Parameters
        ----------

        observations : list
            List of Observation objects.

        idx : int
            The index of the current observation.

        number : int
            Number of prior observations.

        Returns
        -------
        : list
            List of prior Observation objects preceding the current observation.
        """
        if idx > len(observations) - 1:
            raise IndexError("Index not valid for given length of observations sequence.")
        for spread in reversed(range(number + 1)):
            if 0 <= idx - spread < len(observations):
                return observations[idx - spread:idx]
        return []

    def _euclidean(self, obj1_posistion, obj2_position):
        if isinstance(obj1_posistion, (list, np.ndarray)) and isinstance(obj2_position, (list, np.ndarray)):
            return euclidean(obj1_posistion, obj2_position)
        else:
            raise ValueError("Argument must be array-like (list or ndarray)")

    def _average_discrete_velocity(self, distances, timestamps):
        # Takes in a list of distances and timestamps so that velocity is smoothed by using a small window
        # of velocities between prior observations finding the velocity between consecutive observations
        # then averaging across all.
        if len(distances) != len(timestamps):
            raise ValueError("Every distance entry must have a corresponding timestamp")
        velocity_sum = 0
        for idx in range(0, len(distances)):
            if idx != len(distances) - 1:
                update = self._discrete_velocity(distances[idx], distances[idx + 1], timestamps[idx], timestamps[idx + 1])
                if abs(update) < 2.0:
                    velocity_sum += update
        return velocity_sum / int(len(distances) / 2)

    def _discrete_velocity(self, dist_before, dist_after, timestamp_before, timestamp_after):
        return (dist_after - dist_before) / (timestamp_before - timestamp_after)
