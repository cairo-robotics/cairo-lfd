import intera_interface
from predicate_classification.predicate_classifiers import height, upright
from lfd_processor.items import convert_data_to_pose


class HeightConstraint(object):

    def __init__(self, constraint_id, item_id, button, reference_height, threshold_distance):
        self.id = constraint_id
        self.item_id = item_id
        self.reference_height = reference_height
        self.threshold_distance = threshold_distance
        self.button = button

    def check_trigger(self):
        if intera_interface.Navigator().get_button_state(self.button) is not 0:
            return 1
        else:
            return 0

    def evaluate(self, environment, observation):
        if self.item_id is environment.get_robot_info()["id"]:
            item_data = observation.get_robot_data()
            item_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])
        else:
            item_data = observation.get_item_data(self.item_id)
            item_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])

        return height(item_pose, self.reference_height, self.threshold_distance)


class UprightConstraint(object):

    def __init__(self, constraint_id, item_id, button, threshold_angle, axis):
        self.id = constraint_id
        self.item_id = item_id
        self.button = button
        self.threshold_angle = threshold_angle
        self.axis = str(axis)

    def check_trigger(self):
        if intera_interface.Navigator().get_button_state(self.button) is not 0:
            return 1
        else:
            return 0

    def evaluate(self, environment, observation):
        if self.item_id is environment.get_robot_info()["id"]:
            item_data = observation.get_robot_data()
            item_info = environment.get_robot_info()
        else:
            item_data = observation.get_item_data(self.item_id)
            item_info = environment.get_item_info(self.item_id)

        current_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])
        upright_pose = convert_data_to_pose(item_info["upright_pose"]["position"], item_info["upright_pose"]["orientation"])

        return upright(upright_pose, current_pose, self.threshold_angle, self.axis)
