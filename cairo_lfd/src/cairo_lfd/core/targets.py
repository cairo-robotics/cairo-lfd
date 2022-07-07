"""
The items.py module contains container classes for items used in the Cario LfD ecosystem, generally non-robot objects.
"""
from abc import ABCMeta, abstractmethod

import numpy as np
import tf
import rospy
import intera_interface

from robot_clients.transform_clients import TransformLookupClient


class DataTong():
    """
    Class representing the dynamic items in the LFD Environment. These are items that can be moved / altered
    throughout the life cycle of a given experiment / task.

    Attributes
    ----------
    id : int
            Id of item.
    name : str
       Name of the item.
    upright_pose : dict
       Dictionary containing 'position' and 'orientation' keys and corresponding values. The orientation should represent the objects upright pose.
    world_frame : str
        The base / world frame from which to calculate the transformation to the child frame.
    child_frame : str
        The ending TF frame used to generate the pose / orientation of the robot (usually the end effector tip).
    tlc : TransformLookupClient
        The client that makes calls to TransformLookupServer in order to get the transformation between world_frame and child_frame
    """

    def __init__(self, object_id, name="data_tongs", world_frame="world", service_name="transform_lookup_service"):
        """
        Parameters
        ----------
        object_id : int
            Id of item.
        name : str
           Name of the item.
       service_name : str
            Name of transformation lookup service used by _get_transform() to calculate the transformation between world_frame and child_frame
     
        """
        self.id = object_id
        self.name = name
        self.world_frame = world_frame if world_frame is not None else ""
        self.tlc = TransformLookupClient(service_name)

    def get_state(self):
        """
        Gets the item's state.

        Returns
        -------
        state : dict
            The state of the dynamic item
        """
        state = {}
        trans = self._get_transform()
        state['id'] = self.id
        state['position'] = trans["position"]
        state['orientation'] = trans["orientation"]
        return state

    def _get_transform(self):
        """
        Utilizes the tlc (TransformLookupClient) to obtain the transformation between self.world_frame and self.child_frame.

        Returns
        -------
        transform : dict
            Dictionary of representing transformation containing position and orientation keys.
        """
        trans = self.tlc.call(self.world_frame, self.child_frame).transform
        transform = {
            "position": [trans.translation.x, trans.translation.y, trans.translation.z],
            "orientation": [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
        }
        return transform

