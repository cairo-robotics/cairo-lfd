"""
The conversion.py module contains methods and classes that converts data to different types or state spaces.
"""
import copy

import numpy as np
from geometry_msgs.msg import Pose

from cairo_lfd.core.environment import Observation, SimpleObservation


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


class SawyerSampleConversion(object):
    """
    Converts raw samples generated from models into Observation objects.

    The main purpose of this class is to take samples generated from a model and ground them into a usable
    state space for the robot. This could be converting an end-effector relative to object translation sample into the configuration space of the robot, or the pose space of the end-effector. The class is capable of running forward kinematics if needed.

    Attributes
    ----------
    interface : object
        SawyerMoveitInterface to help run forward kinematics.
    """

    def __init__(self, interface, grounded_transform=None):
        """
        Parameters
        ----------
        interface : cairo_robot_interface.base_inteface.AbstractRobotInterface
            SawyerMoveitInterface to help run forward kinematics.
        adapter : object
            An adapter class designed to take raw sample from a model and perform a transformation on the raw sample, generally to ground the object against some other object in the environment.
        """
        self.interface = interface
        self.grounded_transform = grounded_transform

    def convert(self, sample, primal_observation=None, run_fk=False, normalize_quaternion=False):
        """
        Converts raw samples generated from models into Observation objects.

        Parameters
        ----------
        sample : list
            Raw sample to convert.

        primal_observation : Observation
            The most representative observation of the keyframe. This is used to pull into necessary state information (other object positions etc,.) about the environment at the time of the demonstrations. 

        run_fk : bool
            Flag indicating whether to run forward kinematics or not.

        normalize_quaternion : bool
            Flag indicating whether to normalize the values of the quaternion entries.

        Returns
        -------
        obsv : lfd.environment.Observation
            Observation object constructed from the converted sample.
        """
        if self.grounded_transform is not None:
            sample = self.grounded_transform(sample)
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
    
class Holonomic2DSampleConversion(object):
    """
    Converts raw samples generated from models into SimpleObservation objects for a holonomic x, y, theta robot.

    """

    def convert(self, sample):
        """
        Converts raw samples generated from models into Observation objects.

        Parameters
        ----------
        sample : list
            Raw sample to convert.

        Returns
        -------
        obsv : lfd.environment.SimpleObservation
            Simplebservation object constructed from the converted sample.
        """
        data = {
            "robot": {
                "x": sample[0],
                "y": sample[1],
                "theta": sample[2]
            }
        }
        obsv = SimpleObservation(data)
       
        return obsv

    