"""
The items.py module contains container classes for items uses in the Cario LfD ecosystem.
These could include robots, constraints, and environment objects such as a cup etc,.
"""
import rospy
import intera_interface
from abc import ABCMeta, abstractmethod
from lfd.constraints import UprightConstraint, HeightConstraint


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


class SawyerRobot(AbstractItem):
    """
    Class representing the Saywer robot in the LFD Environment object.

    Example
    -------

    upright pose:

    .. code-block:: json

        {
            "position":
                [
                    0.604698787426,
                    -0.439894686226,
                    0.159350584992
                ],
            "orientation":
                [
                    0.712590112587,
                    -0.00994445446764,
                    0.701496927312,
                    -0.00430119065513
                ]
        }

    Attributes
    ----------
    id : int
            Id of robot assigned in the config.json configuration files.
    upright_pose : dict
       Dictionary with position and orientation fields indicating the upright orientation of the Sawyer end-effector.
    _limb : object
        Intera SDK class object that provides controlling functionality of the Sawyer Robot.
    _cuff : object
        Intera SDK class object that provides controlling interface of the cuff bottons of Sawyer robot.
    _navigator : object
        Intera SDK class object that provides controlling functionality of the button/wheel interface on the Sawer Robot.
    _gripper : object
        Intera SDK class object that provides controlling functionalirty of the Sawyer Robot gripper.
    """
    def __init__(self, robot_id, upright_pose):
        """
        Parameters
        ----------
        robot_id : int
            Id of robot assigned in the config.json configuration files.
        upright_pose : dict
           Dictionary with position and orientation fields
        """
        self.id = robot_id
        self.upright_pose = upright_pose
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

    def get_state(self):
        """
        Get's the current state of the robot.

        Example
        -------

        State returned:

        .. code-block:: json

        {
            id: robot_id
            position: [x, y ,z],
            orientation: [x, y, z, w]
            joints: [j0, j1, j2, j3, j4, j5, j6],
            gripper: .123123123123
        }

        Returns
        -------
        state : dict
            The state of the robot
        """

        state = {}
        joints = self._limb.joint_names()
        pose = self._limb.endpoint_pose()
        state["id"] = self.id
        state['position'] = [x for x in pose["position"]]
        state['orientation'] = [x for x in pose["orientation"]]
        state['gripper'] = self._gripper.get_position()
        state['joints'] = [self._limb.joint_angle(j) for j in joints]
        return state

    def get_info(self):
        """
        Get's the robot item's information.

        Returns
        -------
        state : dict
            The info of the robot item

        Example
        -------

        Info returned:

        .. code-block:: json
        {
            id: robot_id
            upright_pose: {
                position: [x, y ,z],
                orientation: [x, y, z, w]
            }
        }
        """
        return {
                    "id": self.id,
                    "upright_pose": self.upright_pose
               }


class RobotFactory(object):
    """
    Factory class that builds robot items. These items are defined in the config.json file.
    The class field in the configuration determines which AbstractItem robot class to use.

    Attributes
    ----------
    configs : list
            List of configuration dictionaries.
    classes : dict
        Dictionary with values as uninitialized class references i.e. SawyerRobot

    Example
    -------

    Example entry in config.json:

    .. code-block:: json

        {
            "class": "SawyerRobot",
            "init_args":
                {
                    "id": 1,
                    "upright_pose":
                        {
                            "position":
                                [
                                    0.604698787426,
                                    -0.439894686226,
                                    0.159350584992
                                ],
                            "orientation":
                                [
                                    0.712590112587,
                                    -0.00994445446764,
                                    0.701496927312,
                                    -0.00430119065513
                                ]
                        }
                }
        }
    """
    def __init__(self, robot_configs):

        """
        Parameters
        ----------
        robot_configs : list
            List of configuration dictionaries.
        """
        self.configs = robot_configs
        self.classes = {
            "SawyerRobot": SawyerRobot,
        }

    def generate_robots(self):
        """
        Build the robots defined in the configuration dictionaries of self.configs.

        Returns
        -------
        robots : list
            List of AbstractItem robot objects.
        """
        robots = []
        for config in self.configs:
            robots.append(self.classes[config["class"]](*tuple(config["init_args"].values())))
        return robots


class ConstraintFactory(object):
    """
    Factory class that builds LFD constraints. These items are defined in the config.json file.
    The class field in the configuration determines which constraint class to use.

    Attributes
    ----------
    configs : list
            List of configuration dictionaries.
    classes : dict
        Dictionary with values as uninitialized class references to constraint classes i.e. HeightConstraint

    Example
    -------

    Example entry in config.json:

    .. code-block:: json

        {
            "class": "HeightConstraint",
            "init_args" :
                {
                    "id": 1,
                    "item": 1,
                    "button": "right_button_square",
                    "reference_height": 0.0,
                    "threshold_distance": 0.25

                }
        }
    """
    def __init__(self, constraint_configs):
        """
        Parameters
        ----------
        constraint_configs : list
            List of configuration dictionaries.
        """
        self.configs = constraint_configs
        self.classes = {
            "UprightConstraint": UprightConstraint,
            "HeightConstraint": HeightConstraint
        }

    def generate_constraints(self):
        """
        Build the constraint objects defined in the configuration dictionaries of self.configs.

        Returns
        -------
        robots : list
            List of constraint objects.
        """
        constraints = []
        for config in self.configs:
            constraints.append(self.classes[config["class"]](*tuple(config["init_args"].values())))
        return constraints
