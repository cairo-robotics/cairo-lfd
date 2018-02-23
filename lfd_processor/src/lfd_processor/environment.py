import json
from collections import OrderedDict


def import_configuration(filepath):

    """
    Wrapper function around json.load() to import a config.json file used to inform the Environment object.
    """

    with open(filepath) as json_data:
        return json.load(json_data, object_pairs_hook=OrderedDict)


class Environment(object):

    """
    HeightConstraint class to evaluate the height predicate classifier assigned to a given item.
    """

    def __init__(self, items, robot, constraints):

        """
        Parameters
        ----------
        items : list
            List of environment items/objects i.e. blocks, spatulasm, cups etc,.. of the AbstractItem class.
        robot : SawyerRobot
            AbstractItem extended class object representing the robot.
        constraints : list
            List of constrain class objects representing the available constraints for the Demonstration.
        """

        self.items = items
        self.robot = robot
        self.constraints = constraints

    def get_robot_state(self):

        """
        Retrieves the robot item's current state

        Returns
        -------
        entries : dict
            Dictionary representing robot's state. See SawyerRobot class.
        """

        return self.robot.get_state()

    def get_robot_info(self):

        """
        Retrieves the robot item's configuration information.

        Returns
        -------
        entries : dict
            Dictionary representing robot's information. See SawyerRobot class.
        """

        return self.robot.get_info()

    def get_item_states(self):

        """
        Retrieves the Environment's items states.

        Returns
        -------
        entries : list
            List of dictionaries for each of the Environment's items (exluding the robot item)
        """

        item_states = []
        if self.items is not None:
            for item in self.items:
                item_states.append(item.get_state())
        return item_states

    def get_item_info(self):

        """
        Retrieves the Environment's items information.

        Returns
        -------
        entries : list
            List of dictionaries for each of the Environment's items (exluding the robot item)
        """

        item_info = []
        if self.items is not None:
            for item in self.items:
                item_info.append(item.get_info())
        return item_info

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

        return [constraint for constraint in self.constraints if constraint.id == constraint_id][0]

    def check_constraint_triggers(self):

        """
        Checks all constraints for their trigger. A triggered constraint might be a button press on Sawyer's
        cuff or a natural language dicated constraint.

        Returns
        -------
        triggered_cosntraints: list
          List of the id's of all the constraints currently triggered
        """

        triggered_constraints = []
        for constraint in self.constraints:
            result = constraint.check_trigger()
            if result != 0:
                triggered_constraints.append(constraint.id)
        return triggered_constraints


class Demonstration(object):

    """
    Demonstration object to contain list of osbervations and various methods to perform on those observations.
    """

    def __init__(self, observations, aligned_observation=None, labeled_observations=None):

        """
        Parameters
        ----------
        observations : list
            List of Observation objects representing raw observations from demonstration. (Requried)
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

    Example:
        {
            "applied_constraints": [],
            "items": [
                {
                    "id": 1,
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
                    "id": 2,
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
                "joints": [
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
    def init_samples(cls, pose, orientation):
        """
        Parameters
        ----------
        pose : array of pose data
        orientation: array of orientation data
        """
        observation_data = {"robot":{"orientation": orientation,
                                     "position": pose}}
        return cls(observation_data)



    def get_timestamp(self):

        """
        Get's osbervations timestamp

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
        Get the pose pose of the observatoin as a list

        Returns
        ------
        : list
            combined data from position and orientation
        """

        robot = self.get_robot_data()
        pose = [robot["position"].append["orientation"]]
        return pose

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

        for item in self.data["items"]:
            # return first occurance, should only be one
            if item["id"] == item_id:
                return item

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
