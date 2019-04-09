"""
The constraints.py module contains a classes that encapsulate predicate classifiers used to
evaluate binary value conceptual constraints.
"""
import intera_interface
from predicate_classification.pose_classifiers import height, upright, over_under
from predicate_classification.path_classifiers import perimeter
from lfd.processing import convert_data_to_pose


class HeightConstraint(object):
    """
    HeightConstraint class to evaluate the height predicate classifier assigned to a given item.

    height() returns true if object distance from reference_height is greater than threshold distnace.

        A positive threshold distance mean height above

        A negative threshold distance means height below.

    Attributes
    ----------
    id : int
        Id of the constraint as defined in the config.json file.
    item_id : int
        Id of the item on which the constraint can be applied.
    reference_height : int
        The reference or starting height to compare an objects height distance against the threshold_distance.
    threshold_distance : int
        The distance from reference (positive: above; negative; below) to compare an object's distance
        from reference.
    button : string
        String of a button for the intera_interface.Navigator().get_button_state(self.button) function to
        check the trigger.
    """
    def __init__(self, constraint_id, item_id, button, reference_height, threshold_distance):

        """
        These arguments should be in the "init_args" field of the config.json file's entry representing
        this constraint.

        Parameters
        ----------
        constraint_id : int
            Id of the constraint as defined in the config.json file.
        item_id : int
            Id of the item on which the constraint can be applied.
        button : string
            String of a button for the intera_interface.Navigator().get_button_state(self.button) function to
            check the trigger.
        reference_height : int
            The reference or starting height to compare an objects height distance against the threshold_distance.
        threshold_distance : int
            The distance from reference (positive: above; negative; below) to compare an object's distance
            from reference.
        """

        self.id = constraint_id
        self.item_id = item_id
        self.reference_height = reference_height
        self.threshold_distance = threshold_distance
        self.button = button

    def check_trigger(self):
        """
        This function evaluates whether the constrain has been triggered. In this case,
        this class's trigger uses the cuff buttons of Sawyer.

        Returns
        -------
        : int
            Boolean value of trigger result.
        """
        if intera_interface.Navigator().get_button_state(self.button) != 0:
            return 1
        else:
            return 0

    def evaluate(self, environment, observation):
        """
        This function evaluates an observation for the assigned constraint of the class. It differentiates
        between Sawyer (end-effector) and general items (blocks etc,.).

        Parameters
        ----------
        environment : Environment
            The Environment object containing the current demonstrations environment (SawyerRobot, Items, Constraints)
            and helper methods.

        observation : Observation
            The observation to evaluate for the constraint.

        Returns
        -------
         : int
            Boolean value of constraint evaluation for the height constraint.
        """
        if self.item_id == environment.get_robot_info()["id"]:
            item_data = observation.get_robot_data()
            item_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])
        else:
            item_data = observation.get_item_data(self.item_id)
            item_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])

        return height(item_pose, self.reference_height, self.threshold_distance)


class UprightConstraint(object):
    """
    Upright Constraint class to evaluate the upright predicate classifier assigned to a given item.

    upright() returns true if object distance is within a threshold angle from its defined upright orientation,
    pivoting around a given axis.

    Attributes
    ----------
    id : int
        Id of the constraint as defined in the config.json file.
    item_id : int
        Id of the item on which the constraint can be applied.
     threshold_angle : int
        The angle within which the assigned item's (from item_id) current orientation must be compared with its
        defined upright position.
    axis : int
        The axis from which angle of deviation is calculated.
    button : string
        String of a button for the intera_interface.Navigator().get_button_state(self.button) function to
        check the trigger.
    """
    def __init__(self, constraint_id, item_id, button, threshold_angle, axis):
        """
        These arguments should be in the "init_args" field of the config.json file's entry representing this constraint.

        Parameters
        ----------
        constraint_id : int
            Id of the constraint as defined in the config.json file.
        item_id : int
            Id of the item on which the constraint can be applied.
        button : string
            String of a button for the intera_interface.Navigator().get_button_state(self.button) function to
            check trigger.
        threshold_angle : int
            The angle within which the assigned item's (from item_id) current orientation must be compared with its
            defined upright position.
        axis : int
            The axis from which angle of deviation is calculated.
        """
        self.id = constraint_id
        self.item_id = item_id
        self.threshold_angle = threshold_angle
        self.axis = str(axis)
        self.button = button

    def check_trigger(self):
        """
        This function evaluates whether the constraint has been triggered. In this case,
        this class's trigger uses the cuff buttons of Sawyer.

        Returns
        -------
        : int
            Boolean value of trigger result.
        """
        if intera_interface.Navigator().get_button_state(self.button) != 0:
            return 1
        else:
            return 0

    def evaluate(self, environment, observation):
        """
        This function evaluates an observation for the assigned constraint of the class. It differentiates
        between Sawyer (end-effector) and general items (blocks etc,.).

        Parameters
        ----------
        environment : Environment
            The Environment object containing the current demonstrations environment (SawyerRobot, Items, Constraints)
            and helper methods.

        observation : Observation
            The observation to evaluate for the constraint.

        Returns
        -------
         : int
            Boolean value of constraint evaluation for the associate constraint and item.
        """
        if self.item_id == environment.get_robot_info()["id"]:
            item_data = observation.get_robot_data()
            item_info = environment.get_robot_info()
        else:
            item_data = observation.get_item_data(self.item_id)
            item_info = environment.get_item_info(self.item_id)

        current_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])
        upright_pose = convert_data_to_pose(item_info["upright_pose"]["position"], item_info["upright_pose"]["orientation"])
        return upright(upright_pose, current_pose, self.threshold_angle, self.axis)


class OverUnderConstraint(object):
    """
    OverUnderConstraint class to evaluate the over_under predicate classifier assigned to a given item.

    over_under() returns true if one pose is above another pose and within the a threshold distance
    in the plane orthogonal to the given axis.

    Attributes
    ----------
    id : int
        Id of the constraint as defined in the config.json file.
    above_item_id : int
            Id of the item that must be above the other for the constraint to hold true.
    below_item_id : int
        Id of the item that must be below the other for the constraint to hold true.
    button : string
        String of a button for the intera_interface.Navigator().get_button_state(self.button) function to
        check the trigger.
    threshold_distance : int
        The distance from reference (positive: above; negative; below) to compare an object's distance
        from reference.
    axis : str
        The axis from which angle of deviation is calculated.
    """
    def __init__(self, constraint_id, above_item_id, below_item_id, button, threshold_distance, axis):

        """
        These arguments should be in the "init_args" field of the config.json file's entry representing
        this constraint.

        Parameters
        ----------
        constraint_id : int
            Id of the constraint as defined in the config.json file.
        above_item_id : int
            Id of the item that must be above the other for the constraint to hold true.
        below_item_id : int
            Id of the item that must be below the other for the constraint to hold true.
        button : string
            String of a button for the intera_interface.Navigator().get_button_state(self.button) function to
            check the trigger.
        threshold_distance : int
            The distance from reference (positive: above; negative; below) to compare an object's distance
            from reference.
        axis : str
            The axis from which angle of deviation is calculated.
        """

        self.id = constraint_id
        self.above_item_id = above_item_id
        self.below_item_id = below_item_id
        self.threshold_distance = threshold_distance
        self.button = button
        self.axis = axis

    def check_trigger(self):
        """
        This function evaluates whether the constrain has been triggered. In this case,
        this class's trigger uses the cuff buttons of Sawyer.

        Returns
        -------
        : int
            Boolean value of trigger result.
        """
        if intera_interface.Navigator().get_button_state(self.button) != 0:
            return 1
        else:
            return 0

    def evaluate(self, environment, observation):
        """
        This function evaluates an observation for the assigned constraint of the class. It differentiates
        between Sawyer (end-effector) and general items (blocks etc,.).

        Parameters
        ----------
        environment : Environment
            The Environment object containing the current demonstrations environment (SawyerRobot, Items, Constraints)
            and helper methods.

        observation : Observation
            The observation to evaluate for the constraint.

        Returns
        -------
         : int
            Boolean value of constraint evaluation for the height constraint.
        """
        if self.above_item_id == environment.get_robot_info()["id"]:
            above_data = observation.get_robot_data()
            above_pose = convert_data_to_pose(above_data["position"], above_data["orientation"])
        if self.below_item_id == environment.get_robot_info()["id"]:
            below_data = observation.get_robot_data()
            below_pose = convert_data_to_pose(below_data["position"], below_data["orientation"])
        else:
            above_data = observation.get_item_data(self.above_item_id)
            above_pose = convert_data_to_pose(above_data["position"], above_data["orientation"])
            below_data = observation.get_item_data(self.below_item_id)
            below_pose = convert_data_to_pose(below_data["position"], below_data["orientation"])
        return over_under(above_pose, below_pose, self.threshold_distance, axis=axis)


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
