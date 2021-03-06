
"""
The robots.py module contains container classes for robots used in the Cario LfD ecosystem.
"""
from abc import ABCMeta, abstractmethod

import numpy as np
import tf
import rospy
import intera_interface

from robot_clients.transform_clients import TransformLookupClient


class AbstractRobot(object):
    """
    Abstract Base class for represent robot in an Environment.
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


class SawyerRobot(AbstractRobot):
    """
    Class representing the Saywer robot in the LFD Environment object.

    Attributes
    ----------
    robot_id : int
            Id of robot assigned in the config.json configuration files.
    reference_pose : dict
       Dictionary with position and orientation fields indicating the 'correct' orientation of the Sawyer end-effector.
    _limb : object
        Intera SDK class object that provides controlling functionality of the Sawyer Robot.
    _cuff : object
        Intera SDK class object that provides controlling interface of the cuff buttons of Sawyer robot.
    _navigator : object
        Intera SDK class object that provides controlling functionality of the button/wheel interface on the Sawer Robot.
    _gripper : object
        Intera SDK class object that provides controlling functionality of the Sawyer Robot gripper.
    base_frame : str
        The base / world frame from which to calculate the transformation to the child frame.
    child_frame : str
        The ending TF frame used to generate the pose / orientation of the robot (usually the end effector tip).
    tlc : TransformLookupClient
        The client that makes calls to TransformLookupServer in order to get the transformation between world_frame and child_frame
    """

    def __init__(self, item_id, reference_pose, base_frame="world", child_frame="right_gripper_tip", service_name="transform_lookup_service"):
        """
        Parameters
        ----------
        robot_id : int
            Id of robot assigned in the config.json configuration files.
        reference_pose : dict
           Dictionary with position and orientation fields
        base_frame : str
            The base / world frame from which to calculate the transformation to the child frame.
        child_frame : str
            The ending TF frame used to generate the pose / orientation of the robot (usually the end effector tip).
        service_name : str
            Name of transformation lookup service used by _get_transform() to calculate the transformation between world_frame and child_frame
        """

        self.id = item_id
        self.reference_pose = reference_pose
        self._limb = intera_interface.Limb("right")
        self._cuff = intera_interface.Cuff("right")
        self._navigator = intera_interface.Navigator()
        try:
            self._gripper = intera_interface.Gripper("right")
            rospy.loginfo("Electric gripper detected.")
            if self._gripper.has_error():
                rospy.loginfo("Gripper error...rebooting.")
                self._gripper.reboot()
            if not self._gripper.is_calibrated():
                rospy.loginfo("Calibrating gripper.")
                self._gripper.calibrate()
        except Exception as e:
            self._gripper = None
            rospy.loginfo("No electric gripper detected.")
        self.base_frame = base_frame
        self.child_frame = child_frame
        self.tlc = TransformLookupClient(service_name)

    def get_state(self):
        """
        Gets the current state of the robot.

        Returns
        -------
        state : dict
            The state of the robot
        """

        state = {}
        trans = self._get_transform()
        state['id'] = self.id
        state['position'] = trans["position"]
        state['orientation'] = trans["orientation"]
        state['endpoint_velocity'] = self._limb.endpoint_velocity()
        state['gripper_position'] = self._gripper.get_position()
        state['gripper_state'] = self._gripper.is_gripping()
        state['joint_angle'] = [self._limb.joint_angle(
            j) for j in self._limb.joint_names()]
        state['joint_velocity'] = [self._limb.joint_velocity(
            j) for j in self._limb.joint_names()]
        return state

    def get_info(self):
        """
        Gets the robot item's information.

        Returns
        -------
        : dict
            The info of the robot item
        """
        return {"id": self.id,
                "upright_pose": self.reference_pose
                }

    def _get_transform(self):
        """
        Utilizes the tlc (TransformLookupClient) to obtain the transformation between self.world_frame and self.child_frame.

        Returns
        -------
        transform : dict
            Dictionary of representing transformation containing position and orientation keys.
        """
        trans = self.tlc.call(self.base_frame, self.child_frame).transform
        transform = {
            "position": [trans.translation.x, trans.translation.y, trans.translation.z],
            "orientation": [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
        }
        return transform


class RobotFactory(object):
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
            "class": "SawyerRobot",
            "name": "Sawyer",
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

    def __init__(self, configs=None):
        """
        Parameters
        ----------
        robot_configs : list
            List of configuration dictionaries for robots.
        items_configs : list
            List of configuration dictionaries for environment objects.
        """
        self.configs = configs
        self.classes = {
            "SawyerRobot": SawyerRobot
        }

    def generate_robots(self):
        """
        Build the robots defined in the configuration dictionaries of self.configs.

        Returns
        -------
        items : list
            List of AbstractItem item objects.
        """
        robot_ids = []
        robots = []
        for config in self.configs:
            if config["init_args"]["item_id"] in robot_ids:
                raise ValueError(
                    "Robots must each have a unique integer 'item_id'")
            else:
                robot_ids.append(config["init_args"]["item_id"])
            try:
                robots.append(
                    self.classes[config["class"]](**config["init_args"]))
            except TypeError as e:
                rospy.logerr("Error constructing {}: {}".format(
                    self.classes[config["class"]], e))
        return robots
