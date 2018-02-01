import json
from collections import OrderedDict
from lfd_environment.constraints import UprightConstraint, HeightConstraint
from lfd_environment.items import SawyerRobot


def import_configuration(filepath):
        with open(filepath) as json_data:
            return json.load(json_data, object_pairs_hook=OrderedDict)


class Environment(object):

    def __init__(self, items, robot, constraints):
        self.items = items
        self.robot = robot
        self.constraints = constraints

    def get_robot_state(self):
        return self.robot.get_state()

    def get_robot_info(self):
        return self.robot.get_info()

    def get_item_states(self):
        item_states = []
        if self.items is not None:
            for item in self.items:
                item_states.append(item.get_state())
        return item_states

    def get_item_info(self):
        item_info = []
        if self.items is not None:
            for item in self.items:
                item_info.append(item.get_info())
        return item_info

    def get_constraint_by_id(self, constraint_id):
        return [constraint for constraint in self.constraints if constraint.id == constraint_id][0]

    def check_constraint_triggers(self):
        triggered_constraints = []
        for constraint in self.constraints:
            result = constraint.check_trigger()
            if result is not 0:
                triggered_constraints.append(constraint.id)
        return triggered_constraints


class Demonstration(object):

    def __init__(self, observations):
        self.observations = observations

    def vectorize_observations(self, keys=["position", "orientation", "joints"]):
        # This is fragile considering the observation data might vary or not have joints etc.
        observation_vectors = []
        for observation in self.observations:
            vector = []
            for key in keys:
                if key in observation.keys():
                    vector.append[observation[key]]
            observation_vectors.append(vector)
        return observation_vectors


class Observation(object):
    def __init__(self, observation_data):
        self.data = observation_data

    def get_timestamp(self):
        return self.data["time"]

    def get_robot_data(self):
        return self.data["robot"]

    def get_item_data(self, item_id):
        for item in self.data["items"]:
            # return first occurance, should only be one
            if item["id"] is item_id:
                return item

    def get_triggered_constraint_data(self):
        return self.data["triggered_constraints"]

    def get_applied_constraint_data(self):
        if "applied_constraints" in self.data.keys():
            return self.data["applied_constraints"]
        else:
            return None


class RobotFactory(object):

    def __init__(self, robot_configs):
        self.configs = robot_configs
        self.classes = {
            "SawyerRobot": SawyerRobot,
        }

    def generate_robots(self):
        robots = []
        for config in self.configs:
            robots.append(self.classes[config["class"]](*tuple(config["init_args"].values())))
        return robots


class ConstraintFactory(object):

    def __init__(self, constraint_configs):
        self.configs = constraint_configs
        self.classes = {
            "UprightConstraint": UprightConstraint,
            "HeightConstraint": HeightConstraint
        }

    def import_configuration(self, filename):
        with open(filename) as json_data:
            self.constraints = json.load(json_data, object_pairs_hook=OrderedDict)["constraints"]

    def generate_constraints(self):
        constraints = []
        for config in self.configs:
            constraints.append(self.classes[config["class"]](*tuple(config["init_args"].values())))
        return constraints