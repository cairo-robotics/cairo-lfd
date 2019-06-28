"""
The vectorization.py module contains methods to vectorize demonstrations/observation data into numpy arrays.
"""
import numpy as np


def vectorize_robot_position(demonstration):

    """
    Vectorizes a demonstration's observations through the union of the
    robot's position.

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
        vectors.append(np.array(position_data))
    return vectors


def vectorize_robot_orientation(demonstration):

    """
    Vectorizes a demonstration's observations through the union of the
    robot's orientation.

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
        orientation_data = observation.data["robot"]["orientation"]
        vectors.append(np.array(orientation_data))
    return vectors


def vectorize_relative_end_effector_position(demonstration, item_id):

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
    return np.array(observation.get_pose_list())


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
    return np.array(observation.get_joint_angle())