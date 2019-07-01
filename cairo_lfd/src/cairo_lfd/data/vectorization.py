"""
The vectorization.py module contains methods to vectorize demonstrations/observation data into numpy arrays.
"""
import numpy as np


def vectorize_demonstration(demonstration, vectorizors=[vectorize_robot_position]):
    """
    Vectorizes a list of observations by iteratively apply each of the vectorizor function
    to generate a concatenated numpy array.

    The list of vectorizors must take in as their arguments Observation objects.

    Parameters
    ----------
    demonstration : Demonstration
        Demonstration object who's Observations will be vectorized.

    Returns
    -------
    vectors : list
        List of numpy arrays representing Observation vectors of Demonstration
    """
    vectors = []
    for observation in demonstration.observations:
        vector = []
        for vectorizor in vectorizors:
            vector = np.concatenate((vector, vectorizor(observaiton)))
        vectors.append(vector)
    return vectors


def vectorize_robot_position(observation):
    """
    Vectorizes an observation using robot position data.

    Parameters
    ----------
    observation : Observation
      Observation to vectorize.

    Returns
    -------
     : ndarray
        Numpy vector
    """
    return np.array(observation.data["robot"]["position"])


def vectorize_robot_orientation(observation):
    """
    Vectorizes an observation using robot orientation data.

    Parameters
    ----------
    observation : Observation
      Observation to vectorize.

    Returns
    -------
     : ndarray
        Numpy vector
    """
    return np.array(observation.data["robot"]["orientation"])


def vectorize_relative_end_effector_position(observaiton, item_id):

    """
    Vectorizes an observation through the union of the
    robot's position and robot's joints.

    Parameters
    ----------
    observation : Observation
      Observation to vectorize.
    item_id : int
        The id of the item to use as the reference position for relative distance.

    Returns
    -------
     : ndarray
        Numpy vector
    """
    return np.array(observation.data["robot"]["realtive_positions"][item_id])


def get_observation_pose_vector(observation):
    """
    Vectorizes a Observation object by obtaining pose data.

    Parameters
    ----------
    observation : Observation
       Observation object to vectorize.

    Returns
    -------
    : ndarray
       Returns numpy array of pose data as the following [x, y, z, q_x, q_y, q_z, q_w]
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
    : ndarray
       Returns numpy array of joint configuration data. Formatting is dependent on robot DOF etc,.
    """
    return np.array(observation.get_joint_angle())