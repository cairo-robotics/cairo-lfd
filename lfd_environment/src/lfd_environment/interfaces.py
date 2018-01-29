import json
from collections import OrderedDict
from lfd_environment.constraints import UprightConstraint, HeightConstraint
from lfd_environment.robot import SawyerRobot


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

    def get_item_states(self):
        item_states = []
        if self.items is not None:
            for item in self.items:
                item_states.append(item.get_state())
        return item_states

    def check_constraint_triggers(self):
        triggered_constraints = []
        for constraint in self.constraints:
            result = constraint.check_trigger()
            if result is not 0:
                triggered_constraints.append(constraint.id)
        return triggered_constraints

    def get_item_constraints(self, object_id):
        constraints = []
        for constraint in self.constraints:
            if constraint.item == object_id:
                constraints.append(constraint)
        return constraints


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
