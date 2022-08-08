"""
The items.py module contains container classes for items used in the Cario LfD ecosystem, generally non-robot objects.
"""
from abc import ABCMeta, abstractmethod
from scipy.spatial.distance import euclidean


import numpy as np
import tf
import rospy
import intera_interface

from robot_clients.transform_clients import TransformLookupClient


class DataTong():
    """
    Class representing the data tongs object for use in augmented realiaty based robot demonstration.

    Attributes
    ----------
    object_id : int
        Id of item.
    name : str
        Name of the target.
    world_frame : str
    The base / world frame from which to calculate the transformation to the child frame.
    left_child_frame : str
        The ending TF frame used to generate the pose / orientation of the left side of the tong.
    right_child_frame : str
        The ending TF frame used to generate the pose / orientation of the right side of the tong.
    service_name : str
        Name of transformation lookup service used by _get_transform() to calculate the transformation between world_frame and child_frame
    """

    def __init__(self, name="data_tong", world_frame="world", left_child_frame="left_data_tong", right_child_frame="right_data_tong", service_name="transform_lookup_service", closed_epsilon=.1):
        """
        Parameters
        ----------
        object_id : int
            Id of item.
        name : str
           Name of the target.
        world_frame : str
        The base / world frame from which to calculate the transformation to the child frame.
        left_child_frame : str
            The ending TF frame used to generate the pose / orientation of the left side of the tong.
        right_child_frame : str
            The ending TF frame used to generate the pose / orientation of the right side of the tong.
        service_name : str
            Name of transformation lookup service used by _get_transform() to calculate the transformation between world_frame and child_frame
     
        """
        self.name = name
        self.world_frame = world_frame if world_frame is not None else ""
        self.left_child_frame = left_child_frame
        self.right_child_frame = right_child_frame
        self.epsilon = closed_epsilon
        self.tlc = TransformLookupClient(service_name)

    def get_state(self):
        """
        Gets the data tong's state.

        Returns
        -------
        state : dict
            The state of the data tong.
        """
        state = {}
        left_trans, right_trans = self._get_transforms()
        # we treat position as the midpoint between left and right tong markers.
        mid_point = self._get_midpoint(left_trans["position"], right_trans["position"])
        closed = self._test_closed(left_trans["position"], right_trans["position"])
        state['position'] = mid_point
        # for now we use the right orientation
        state['orientation'] = right_trans["orientation"]
        state['gripper_state'] = closed
        return state
    
    def _test_closed(self, left_pos, right_pos):
        left_xyz = [left_pos['x'], left_pos['y'], left_pos['z']]
        right_xyz = [right_pos['x'], right_pos['y'], right_pos['z']]
        if euclidean(left_xyz, right_xyz) < self.epsilon:
            True
        else:
            False
            
    def _get_midpoint(self, left_pos, right_pos):
        return {'x': (left_pos['x'] + right_pos['x']) / 2, 'y': (left_pos['y'] + right_pos['y']) / 2, 'z': (left_pos['z'] + right_pos['z']) / 2}
        

    def _get_transforms(self):
        """
        Utilizes the tlc (TransformLookupClient) to obtain the transformation between self.world_frame and the left and right child frames.

        Returns
        -------
        transform : dict
            Dictionary of representing transformation containing position and orientation keys.
        """
        left_trans = self.tlc.call(self.world_frame, self.left_child_frame).transform
        l_pos = {}
        l_orientation = {}
        l_pos["x"] = left_trans.translation.x
        l_pos["y"] = left_trans.translation.y
        l_pos["z"] = left_trans.translation.z
        l_orientation["w"] = left_trans.rotation.w
        l_orientation["x"] = left_trans.rotation.x
        l_orientation["y"] = left_trans.rotation.y
        l_orientation["z"] = left_trans.rotation.z
        left_transform = {
            "position": l_pos,
            "orientation": l_orientation
        }
        right_trans = self.tlc.call(self.world_frame, self.right_child_frame).transform
        r_pos = {}
        r_orientation = {}
        r_pos["x"] = right_trans.translation.x
        r_pos["y"] = right_trans.translation.y
        r_pos["z"] = right_trans.translation.z
        r_orientation["w"] = right_trans.rotation.w
        r_orientation["x"] = right_trans.rotation.x
        r_orientation["y"] = right_trans.rotation.y
        r_orientation["z"] = right_trans.rotation.z
        right_transform = {
            "position": r_pos,
            "orientation": r_orientation
        }
        
        return left_transform, right_transform

