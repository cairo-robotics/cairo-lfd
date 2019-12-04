"""
The vectorization.py module contains methods to vectorize demonstrations/observation data into numpy arrays.
"""
import numpy as np
from scipy.spatial.distance import euclidean


def vectorize_demonstration(demonstration, vectorizors=[]):
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
            vector = np.concatenate((vector, vectorizor(observation)))
        vectors.append(vector)
    return np.array(vectors)


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


def xy_radial_distance(observation, above_item_id, below_item_id):
    """
    Vectorizes an observation by comparing xy radial/planer distance between two objects.

    Parameters
    ----------
    observation : Observation
      Observation to vectorize.

    Returns
    -------
     : ndarray
        Numpy vector of shape (1,)
    """
    xy_above = observation.get_item_data(above_item_id)['position'][0:1]
    xy_below = observation.get_item_data(below_item_id)['position'][0:1]
    return np.array([euclidean(xy_above, xy_below)])


def boolean_within_SOI(observation, item1_id, item2_id):
    """
    Vectorizes an observation to a boolean if two objects are within each others' sphere of influence (SOI).

    Depends on the SphereOfInfluenceProcessor object having processed the observation and will simply evaluate to False if this is not the case.

    Parameters
    ----------
    observation : Observation
      Observation to vectorize.

    Returns
    -------
     : bool
        Numpy vector of shape (1,)
    """
    if item2_id in observation.get_item_data(item1_id).get('in_SOI', []):
        return True
    else:
        return False


def boolean_within_perimeter(observation, perimeter_item_id, traversing_item_id):
    """
    Vectorizes an observation to a boolean if traversiding_item is within the perimeter of the
    perimeter item.

    Depends on the SphereOfInfluenceProcessor object having processed the observation and will simply evaluate to False if this is not the case.

    Parameters
    ----------
    observation : Observation
      Observation to vectorize.

    Returns
    -------
     : bool
        Numpy vector of shape (1,)
    """
    if traversing_item_id in observation.get_item_data(perimeter_item_id).get('in_perimeter', []):
        return True
    else:
        return False


def vectorize_relative_end_effector_position(observation, item_id):
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
    return np.array(observation.data["robot"]["relative_positions"][item_id])


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
