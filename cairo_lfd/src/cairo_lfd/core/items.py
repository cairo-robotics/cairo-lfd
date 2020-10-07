"""
The items.py module contains container classes for items used in the Cario LfD ecosystem, generally non-robot objects.
"""
from abc import ABCMeta, abstractmethod

import numpy as np
import tf
import rospy
import intera_interface

from robot_clients.transform_clients import TransformLookupClient


class AbstractItem(object):
    """
    Abstract Base class for represent items in an Environment.
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def get_state(self):
        """
        Abstract method to get the Item's state.
        """
        pass

    @abstractmethod
    def get_info(self):
        """
        Abstract method to get the Item's info.
        """
        pass

class StaticItem(AbstractItem):
    """
    Class representing the static items in the LFD Environment. These are items that will not move
    throughout the life cycle of a given experiment / task.

    Attributes
    ----------
    id : int
            Id of item.
    name : str
       Name of the item.
    pose : dict
       Dictionary containing 'position' and 'orientation' keys and corresponding list of coordinate or orientation values.
    perimeter : dict
        Dictionary containing 'inner' and 'outer' keys corresponding to lists of coordinates representing the inner perimeter and outer perimeter band around the static item relative to it's pose. This assumes the pose is the 'center' of the item.
    """

    def __init__(self, item_id, name, pose, perimeter=None):
        """
        Parameters
        ----------
        object_id : int
                Id of item.
        name : str
           Name of the item.
        pose : dict
           Dictionary containing 'position' and 'orientation' keys and corresponding values. The orientation should represent the objects upright pose.
        perimeter : dict
            Dictionary containing 'inner' and 'outer' keys corresponding to lists of coordinates representing the inner perimeter band and outer perimeter band around the static item.
        """
        self.id = item_id
        self.name = name
        self.pose = pose
        self.perimeter = perimeter

    def get_state(self):
        """
        Gets the static item's state.

        Returns
        -------
        state : dict
            The state of the static item
        """
        state = {}
        state['id'] = self.id
        state['position'] = self.pose["position"]
        state['orientation'] = self.pose["orientation"]
        state['perimeter'] = self.perimeter
        return state

    def get_info(self):
        """
        Gets the item's information.

        Returns
        -------
        : dict
            The info of the static item
        """
        info = {}
        info["id"] = self.id
        info["name"] = self.name
        return info


class DynamicItem(AbstractItem):
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
    perimeter : dict
        Dictionary containing 'inner' and 'outer' keys corresponding to lists of coordinates representing the inner perimeter band 
        and outer perimeter band around the static item.
    """

    def __init__(self, object_id, name, upright_pose, world_frame, child_frame, perimeter=None, service_name="transform_lookup_service"):
        """
        Parameters
        ----------
        object_id : int
                Id of item.
        name : str
           Name of the item.
        upright_pose : dict
           Dictionary containing 'position' and 'orientation' keys and corresponding values. The orientation should represent the objects upright pose.
        world_frame : str
            The base / world frame from which to calculate the transformation to the child frame.
        child_frame : str
            The ending TF frame used to generate the pose / orientation of the robot (usually the end effector tip).
       service_name : str
            Name of transformation lookup service used by _get_transform() to calculate the transformation between world_frame and child_frame
        perimeter : dict
            Dictionary containing 'inner' and 'outer' keys corresponding to lists of coordinates representing the inner perimeter band 
            and outer perimeter band around the static item.
        """
        self.id = object_id
        self.name = name
        self.upright_pose = upright_pose
        self.world_frame = world_frame if world_frame is not None else ""
        self.child_frame = child_frame if world_frame is not None else ""
        self.tlc = TransformLookupClient(service_name)
        self.perimeter = perimeter

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

    def get_info(self):
        """
        Gets the item's information.

        Returns
        -------
        : dict
            The info of the dynamic item
        """
        info = {}
        info["id"] = self.id
        info["name"] = self.name
        return info

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


class ItemFactory(object):
    """
    Factory class that builds environment items. These items are defined in the config.json file.
    The class field in the configuration determines which AbstractItem object class to use.

    Attributes
    ----------
    configs : list
            List of configuration dictionaries.
    classes : dict
        Dictionary with values as uninitialized class references i.e. StaticItem, DynamicItem, SawyerRobot

    Example
    -------

    Example entry in config.json:

    .. code-block:: json

        {
            "class": "StaticItem",
            "name": "Block1",
            "init_args":
                {
                    "object_id": 1,
                    "upright_pose":
                        {
                            "position":
                                [
                                    0.604698787426,
                                    -0.439894686226,
                                    0.159350584992
                                ],
                            "orientation": [
                                0.712590112587,
                                -0.00994445446764,
                                0.701496927312,
                                -0.00430119065513
                            ]
                        },
                    "world_frame": "world",
                    "child_frame": "block"
                }
        }
    """

    def __init__(self, configs):
        """
        Parameters
        ----------
        configs : list
            List of configuration dictionaries for environment objects.
        """
        self.configs = configs
        self.classes = {
            "StaticItem": StaticItem,
            "DynamicItem": DynamicItem
        }

    def generate_items(self):
        """
        Build the items defined in the configuration dictionaries of self.configs.

        Returns
        -------
        items : list
            List of AbstractItem item objects.
        """
        item_ids = []
        items = []
        print(self.configs)
        for config in self.configs:
            print(config)
            if config["init_args"]["item_id"] in item_ids:
                raise ValueError(
                    "Items must each have a unique integer 'item_id'")
            else:
                item_ids.append(config["init_args"]["item_id"])
            try:
                items["items"].append(
                    self.classes[config["class"]](**config["init_args"]))
            except TypeError as e:
                rospy.logerr("Error constructing {}: {}".format(
                    self.classes[config["class"]], e))
        return items
