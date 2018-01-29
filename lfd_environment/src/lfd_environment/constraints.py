import intera_interface


class HeightConstraint(object):

    def __init__(self, constraint_id, item, button, reference_height, threshold_distance):
        self.id = constraint_id
        self.item = item
        self.reference_height = reference_height
        self.threshold_distance = threshold_distance
        self.button = button

    def check_trigger(self):
        if intera_interface.Navigator().get_button_state(self.button) is not 0:
            return 1
        else:
            return 0


class UprightConstraint(object):

    def __init__(self, constraint_id, item, button, threshold_angle):
        self.id = constraint_id
        self.item = item
        self.button = button

    def check_trigger(self):
        if intera_interface.Navigator().get_button_state(self.button) is not 0:
            return 1
        else:
            return 0
