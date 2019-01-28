"""
The processing.py module contains classes and methods for manipulating data/objects into difference forms.
"""
import numpy as np
from geometry_msgs.msg import Pose
from environment import Observation
from scipy.spatial.distance import euclidean
from itertools import combinations


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
        for obsv in observations:
            self.relative_distance(obsv)

    def relative_distance(self, observation):
        # relative to items
        for item_id in self.item_ids:
            relative_distances = {}
            item_data = observation.get_item_data(item_id)
            item_position = item_data["position"]
            for target_id in self.item_ids:
                if item_id != target_id:
                    target_position = observation.get_item_data(target_id)["position"]
                    relative_distances[target_id] = self._euclidean(item_position, target_position)
            if item_id != self.robot_id:
                robot_position = observation.get_robot_data()["position"]
                relative_distances[self.robot_id] = self._euclidean(item_position, robot_position)
            item_data["relative_distance"] = relative_distances

        # relative to robots

        relative_distances = {}
        robot_data = observation.get_robot_data()
        robot_position = robot_data["position"]
        for item_id in self.item_ids:
            if item_id != self.robot_id:
                item_position = observation.get_item_data(item_id)["position"]
                relative_distances[item_id] = self._euclidean(robot_position, item_position)
        robot_data["relative_distance"] = relative_distances

    def relative_velocity(self, obj1_window, obj2_window):
        pass

    def relative_acceleration(self, obj1_window, obj2_window):
        pass

    def _get_window(self):
        pass  

    def _euclidean(self, obj1_posistion, obj2_position):
        if isinstance(obj1_posistion, (list, np.ndarray)) and isinstance(obj2_position, (list, np.ndarray)):
            return euclidean(obj1_posistion, obj2_position)
        else:
            raise ValueError("Argument must be array-like (list or ndarray)")
