"""
The conversion.py module contains methods and classes that converts data to different types or state spaces.
"""
import copy
import numpy as np
from geometry_msgs.msg import Pose
from environment import Observation


def vectorize_demonstration(demonstration):

    """
    Vectorizes a demonstration's observations through the union of the
    robot's position and robot's joints.

    Parameters
    ----------
    demonstration : Demonstration
      Demonstrations to vectorize.

    Returns
    -------
    vectors : list
        List of observation vectors.
    """

    vectors = []
    for observation in demonstration.observations:
        position_data = observation.data["robot"]["position"]
        vector = position_data
        vectors.append(vector)
    return vectors


def get_observation_pose_vector(observation):
    """
    Vectorizes a Observation object by obtaining pose data.

    Parameters
    ----------
    observation : Observation
       Observation object to vectorize.

    Returns
    -------
    : list
       Returns list of pose data as the following [x, y, z, q_x, q_y, q_z, q_w]
    """
    return observation.get_pose_list()


def get_observation_joint_vector(observation):
    """
    Vectorizes a Observation object by obtaining joint configuration data.

    Parameters
    ----------
    observation : Observation
       Observation object to vectorize.

    Returns
    -------
    : list
       Returns list of joint configuration data. Formatting is dependent on robot DOF etc,.
    """
    return observation.get_joint_angle()


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

    def convert(self, sample, primal_observation=None, run_fk=False, normalize_quaternion=False):
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
            sample[3], sample[4], sample[5], sample[6] = self._normalize_quaternion(
                sample[3], sample[4], sample[5], sample[6])

        if len(sample) > 7:
            # If length > 7, we know there must be joint data, so creat Obs w/ joints.
            position = sample[0:3]
            orientation = sample[3:7]
            joints = sample[7:14]
        else:
            position = sample[0:3]
            orientation = sample[3:7]
            joints = None
        if primal_observation is not None:
            obsv = copy.deepcopy(primal_observation)
            obsv.data["robot"]["position"] = position
            obsv.data["robot"]["orientation"] = orientation
            obsv.data["robot"]["joint_angle"] = joints
        else:
            obsv = Observation.init_samples(position, orientation, joints)
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

