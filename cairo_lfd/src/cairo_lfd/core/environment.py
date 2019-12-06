"""
The environment.py module contains core data container classes and retrieval methods. The core classes
are the Environment, Demonstration, and Observation used throughout Cairo LfD's code base.
"""
import json
import numpy as np
from collections import OrderedDict
from geometry_msgs.msg import Pose


class Environment(object):
    """
    Environment container class that houses various objects (items, robots, constraints) relevant to conducting 
    LfD experimentation.

    Attributes
    ----------
    items : list
        List of environment items/objects i.e. blocks, spatulas, cups etc,.. of the AbstractItem class.
    robot : SawyerRobot
        AbstractItem extended class object representing the robot.
    constraints : list
        List of constrain class objects representing the available constraints for the Demonstration.
    triggers : list
        List of trigger class objects that represent when a user has triggered a constraint.
    """
    def __init__(self, items, robot, constraints, triggers):
        """
        Parameters
        ----------
        items : list
            List of environment items/objects i.e. blocks, spatulasm cups etc,.. of the AbstractItem class.
        robot : SawyerRobot
            AbstractItem extended class object representing the robot.
        constraints : list
            List of constrain class objects representing the available constraints for the Demonstration.
        """
        self.items = items
        self.robot = robot
        self.constraints = constraints
        self.triggers = triggers

    def get_robot_state(self):
        """
        Retrieves the robot item's current state

        Returns
        -------
        entries : dict
            Dictionary representing robot's state. See SawyerRobot class.
        """
        if self.robot is not None:
            return self.robot.get_state()
        else:
            raise EnvironmentError("There is no robot configured into the environment!")

    def get_robot_info(self):
        """
        Retrieves the robot item's configuration information.

        Returns
        -------
        entries : dict
            Dictionary representing robot's information. See SawyerRobot class.
        """
        if self.robot is not None:
            return self.robot.get_info()
        else:
            raise EnvironmentError("There is no robot configured into the environment!")

    def get_item_state(self):
        """
        Retrieves the Environment's items states.

        Returns
        -------
        entries : list
            List of dictionaries for each of the Environment's items (excluding the robot item)
        """
        item_states = []
        if self.items is not None:
            for item in self.items:
                item_states.append(item.get_state())
            return item_states
        else:
            raise EnvironmentError("There are no items configured into the environment!")

    def get_item_state_by_id(self, item_id):
        if self.items is not None:
            return [item for item in self.items if item.id == item_id][0]
        else:
            raise EnvironmentError("There are no items configured into the environment!")

    def get_item_info(self):
        """
        Retrieves the Environment's items information.

        Returns
        -------
        entries : list
            List of dictionaries for each of the Environment's items (excluding the robot item)
        """
        item_info = []
        if self.items is not None:
            for item in self.items:
                item_info.append(item.get_info())
            return item_info
        else:
            raise EnvironmentError("There are no items configured into the environment!")

    def get_item_ids(self):
        """
        Retrieves the all AbstractItem id's in the environment exluding Robots.

        Returns
        -------
        ids : list
            List of ids of the Environment's items
        """
        ids = []
        for item in self.items:
            ids.append(item.id)
        ids.sort(reverse=False)
        return ids

    def get_robot_id(self):
        """
        Retrieves environment's robot id.

        Returns
        -------
        id : int
            Id of the environment's robot.
        """
        if self.robot is not None:
            return self.robot.id
        else:
            raise EnvironmentError("There is no robot configured into the environment!")

    def get_constraint_by_id(self, constraint_id):
        """
        Retrieves a constraint from the Environment by it's id.

        Parameters
        ----------
        iconstraint_id : int
            The id of the constraint to retrieve.

        Returns
        -------
        : Constraint class
            Constraint class for the given id.
        """
        if self.constraints is not None:
            return [constraint for constraint in self.constraints if constraint.id == constraint_id][0]
        else:
            raise EnvironmentError("There are no items configured into the environment!")

    def check_constraint_triggers(self):
        """
        Checks all constraints for their trigger. A triggered constraint might be a button press on Sawyer's cuff, a subscribed topic listening for web interface initiated triggers etc,.

        Returns
        -------
        triggered_cosntraints: list
          List of the id's of all the constraints currently triggered
        """
        triggered_constraints = []
        for trigger in self.triggers:
            result = trigger.check()
            if result != 0:
                triggered_constraints.append(trigger.constraint_id)
        return triggered_constraints


class Demonstration(object):
    """
    Demonstration object to contain list of observations and various methods to perform on those observations.

    Attributes
    ----------
    observations : list
        List of Observation objects representing raw observations from demonstration. (Required)
    aligned_observation : SawyerRobot
        List of Observation objects representing DTW aligned observations.
    labeled_observations : list
        List of Observation objects representing keyframe labeled observations.
    """
    def __init__(self, observations, aligned_observation=None, labeled_observations=None):
        """
        Parameters
        ----------
        observations : list
            List of Observation objects representing raw observations from demonstration. (Required)
        aligned_observation : SawyerRobot
            List of Observation objects representing DTW aligned observations.
        labeled_observations : list
            List of Observation objects representing keyframe labeled observations.
        """
        self.observations = observations
        self.aligned_observation = aligned_observation
        self.labeled_observations = labeled_observations

    def get_observation_by_index(self, idx):
        """
        Returns an raw observation by its index.

        Parameters
        ----------
        idx : int
            Index integer

        Returns
        -------
        : Observation
            The retrieved observation object.
        """
        return self.observations[idx]

    def get_applied_constraint_order(self):
        """
        Returns the applied constraint order of a Demonstration's aligned observation list.

        Returns
        -------
        constraint_order: list
            List of list where each element is ordered by the sequence of the applied constraints and represents
            the set of constraints applied.
        """
        constraint_order = []
        curr = []
        for ob in self.aligned_observations:
            if curr != ob.data["applied_constraints"]:
                constraint_order.append(ob.data["applied_constraints"])
                curr = ob.data["applied_constraints"]
        return constraint_order


class Observation(object):
    """
    Observation object to contain raw dictionary data for an observation and retrieval methods for that data.

    Example
    -------
    .. code-block:: json

        {
            "applied_constraints": [],
            "items": [
                {
                    "id": 2,
                    "orientation": [
                        0.6874194527002508,
                        -0.06937214001305077,
                        0.7140914218104518,
                        0.1127627754891884
                    ],
                    "position": [
                        0.8263962716618255,
                        0.3449914200419018,
                        -0.1353068521650853

                },
                {
                    "id": 3,
                    "orientation": [
                        0.7874194527002508,
                        -0.16937214001305077,
                        0.23140914218104518,
                        0.2327627754891884
                    ],
                    "position": [
                        0.1034962716618255,
                        0.9469914200419018,
                        -0.54330685216508534
                    ]
                }
            ],
            "keyframe_id": null, => labeled_demosntration only
            "keyframe_type": null,  => labeled_demosntration only
            "robot": {
                "gripper": 0.0,
                "id": 1,
                "joint_angle": [
                    -0.1242646484375,
                    0.1710810546875,
                    3.044984375,
                    -1.006376953125,
                    -2.976296875,
                    -1.216076171875,
                    -1.41415234375
                ],
                "orientation": [
                    0.6874194527002508,
                    -0.06937214001305077,
                    0.7140914218104518,
                    0.11276277548918841
                ],
                "position": [
                    0.8263962716618255,
                    0.3449914200419018,
                    -0.13530685216508534
                ]
            },
            "time": 0.38476085662841797,
            "triggered_constraints": []
        }

    Attributes
    ----------
    data : dict
        Dictionary of raw data collecting item, robot and constraint states.

    """

    def __init__(self, observation_data):
        """
        Parameters
        ----------
        observation_data : dict
            Dictionary of raw data collecting item, robot and constraint states.
        """
        self.data = observation_data

    @classmethod
    def init_samples(cls, position, orientation, joints_angles):
        """
        Parameters
        ----------
        position : array of position data
        orientation: array of orientation data
        joints_angles: array of joint angles (configuration)
        """
        observation_data = {"robot": {"orientation": orientation,
                                      "position": position,
                                      "joint_angle": joints_angles}}
        return cls(observation_data)

    def get_timestamp(self):
        """
        Gets observations timestamp

        Returns
        -------
        : float
            Integer timestamp in milliseconds representing time from epoch.
        """
        return self.data["time"]

    def get_robot_data(self):
        """
        Get the observation's robot data.

        Returns
        -------
        : dictionary
            dictionary of robot data
        """
        return self.data["robot"]

    def get_pose_list(self):
        """
        Get the pose data of the observation as a list of numerical values.

        Returns
        -------
        : list
            combined data from position and orientation
        """
        robot = self.get_robot_data()
        if isinstance(robot["orientation"], np.ndarray) and isinstance(robot["orientation"], np.ndarray):
            return np.concatenate((robot["position"], robot["orientation"]))
        else:
            return robot["position"] + robot["orientation"]

    def get_joint_angle(self):
        """
        Get the joint angles of the observation as a list of numerical values.

        Returns
        -------
        : list
            list of numerical joint angle values for the given robot.
        """
        robot = self.get_robot_data()
        return robot["joint_angle"]

    def get_pose_msg(self):
        """
        Get pose data and return a pose msg type

        Returns:
        --------
        : geometry_msg Pose
            a pose message of the observation
        """
        pose_list = self.get_pose_list()
        pose_msg = Pose()
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]

        pose_msg.orientation.x = pose_list[3]
        pose_msg.orientation.y = pose_list[4]
        pose_msg.orientation.z = pose_list[5]
        pose_msg.orientation.w = pose_list[6]
        return pose_msg

    def get_keyframe_info(self):
        """
        send back tuple of keyframe num and type

        Returns
        -------
        : tuple
            (keyframe_id, keyframe_type)
        """
        return (self.data["keyframe_id"], self.data["keyframe_type"])

    def get_item_data(self, item_id):
        """
        Gets an items information based on its id

        Parameters
        ----------
        observation_data : dict
            Dictionary of raw data collecting item, robot and constraint states.

        Returns
        -------
        : float
            Integer timestamp in milliseconds representing time from epoch.
        """
        if item_id == self.data["robot"]["id"]:
            return self.get_robot_data()
        items = self.data["items"]
        for item in items:

            # return first occurrence, should only be one
            if item["id"] == item_id:
                return item
        return None

    def get_triggered_constraint_data(self):
        """
        Get the triggered constraints of an object.

        Returns
        -------
        : list
            List of constraint id's.
        """
        return self.data["triggered_constraints"]

    def get_applied_constraint_data(self):
        """
        Get the applied constraints of an object.

        Returns
        -------
        : list
            List of constraint id's.
        """
        if "applied_constraints" in self.data.keys():
            return self.data["applied_constraints"]
        else:
            return None


class ConfigurationError(Exception):

    def __init__(self, value):
        self.value = value

    def __str__(self): 
        return(repr(self.value))


class EnvironmentError(Exception):

    def __init__(self, value):
        self.value = value

    def __str__(self): 
        return(repr(self.value))
