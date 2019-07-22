"""
The constraints.py module contains a classes that encapsulate predicate classifiers used to
evaluate binary value conceptual constraints.
"""
import intera_interface

from predicate_classification.pose_classifiers import height, upright, over_under
from predicate_classification.path_classifiers import perimeter_2D
from cairo_lfd.data.conversion import convert_data_to_pose


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
    """
    def __init__(self, constraint_id, item_id, reference_height, threshold_distance, direction="positive", axis="z"):

        """
        These arguments should be in the "init_args" field of the config.json file's entry representing
        this constraint.

        Parameters
        ----------
        constraint_id : int
            Id of the constraint as defined in the config.json file.
        item_id : int
            Id of the item on which the constraint can be applied.
        reference_height : int
            The reference or starting height to compare an objects height distance against the threshold_distance.
        threshold_distance : int
            The distance from reference (positive: above; negative; below) to compare an object's distance from reference.
        direction : str
            The direction of relative to the axis to evaluate the constraint. 'positive' indicates positive direction along chosen axis or height above. 'negative' means negative direction along chosen axis.
        axis : str
            Axis to evaluate constraint (x, y, or z)
        """

        self.id = constraint_id
        self.item_id = item_id
        self.reference_height = reference_height
        self.threshold_distance = threshold_distance
        self.direction = direction
        self.axis = axis

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
            Integer value of constraint evaluation for the height constraint.
        """
        if self.item_id == environment.get_robot_info()["id"]:
            item_data = observation.get_robot_data()
            item_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])
        else:
            item_data = observation.get_item_data(self.item_id)
            item_pose = convert_data_to_pose(item_data["position"], item_data["orientation"])

        return height(item_pose, self.reference_height, self.threshold_distance, direction=self.direction, axis=self.axis)


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
    """
    def __init__(self, constraint_id, item_id, threshold_angle, axis):
        """
        These arguments should be in the "init_args" field of the config.json file's entry representing this constraint.

        Parameters
        ----------
        constraint_id : int
            Id of the constraint as defined in the config.json file.
        item_id : int
            Id of the item on which the constraint can be applied.
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
            Integer value of constraint evaluation for the associate constraint and item.
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
    threshold_distance : int
        The distance from reference (positive: above; negative; below) to compare an object's distance
        from reference.
    axis : str
        The axis from which angle of deviation is calculated.
    """
    def __init__(self, constraint_id, above_item_id, below_item_id, threshold_distance, axis):

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
        self.axis = str(axis)

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
            Integer value of constraint evaluation for the height constraint.
        """
        if self.above_item_id == int(environment.get_robot_info()["id"]):
            above_data = observation.get_robot_data()
            above_pose = convert_data_to_pose(above_data["position"], above_data["orientation"])
            below_data = observation.get_item_data(self.below_item_id)
            below_pose = convert_data_to_pose(below_data["position"], below_data["orientation"])
        elif self.below_item_id == int(environment.get_robot_info()["id"]):
            above_data = observation.get_item_data(self.above_item_id)
            above_pose = convert_data_to_pose(above_data["position"], above_data["orientation"])
            below_data = observation.get_robot_data()
            below_pose = convert_data_to_pose(below_data["position"], below_data["orientation"])
        else:
            above_data = observation.get_item_data(self.above_item_id)
            above_pose = convert_data_to_pose(above_data["position"], above_data["orientation"])
            below_data = observation.get_item_data(self.below_item_id)
            below_pose = convert_data_to_pose(below_data["position"], below_data["orientation"])
        return over_under(above_pose, below_pose, self.threshold_distance, axis=self.axis)


class Perimeter2DConstraint(object):
    """
    Perimeter class to evaluate the perimeter predicate classifier assigned to a given item.

    over_under() returns true if one pose is above another pose and within the a threshold distance
    in the plane orthogonal to the given axis.

    Attributes
    ----------
    id : int
        Id of the constraint as defined in the config.json file.
    perimeter_item_id : int
        Id of the item for which the perimeter constraint is evaluated.
    traversing_item_id : int
        Id of the item that must that traversing within the perimeter band of the perimeter_item.
    axis : str
        The axis to which the plane of the 2D perimeter of the object is orthogonal.
    """
    def __init__(self, constraint_id, perimeter_item_id, traversing_item_id, axis='z'):

        """
        These arguments should be in the "init_args" field of the config.json file's entry representing
        this constraint.

        Parameters
        ----------
         id : int
        Id of the constraint as defined in the config.json file.
        perimeter_item_id : int
            Id of the item for which the perimeter constraint is evaluated.
        traversing_item_id : int
            Id of the item that must that traversing within the perimeter band of the perimeter_item.
        axis : str
            The axis to which the plane of the 2D perimeter of the object is orthogonal.
        """

        self.id = constraint_id
        self.perimeter_item_id = int(perimeter_item_id)
        self.traversing_item_id = int(traversing_item_id)
        self.axis = str(axis)

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
            Integer value of constraint evaluation for the perimeter_2D constraint.
        """
        if self.traversing_item_id == int(environment.get_robot_info()["id"]):
            traversing_item_data = observation.get_robot_data()
            traversing_item_pose = convert_data_to_pose(traversing_item_data["position"], traversing_item_data["orientation"])
            perimeter_item_data = observation.get_item_data(self.perimeter_item_id)
            inner_poly = perimeter_item_data['perimeter']['inner']
            outer_poly = perimeter_item_data['perimeter']['outer']

        else:
            traversing_item_data = observation.get_item_data(self.traversing_item_id)
            traversing_item_pose = convert_data_to_pose(traversing_item_data["position"], traversing_item_data["orientation"])
            perimeter_item_data = observation.get_item_data(self.perimeter_item_id)
            inner_poly = perimeter_item_data['perimeter']['inner']
            outer_poly = perimeter_item_data['perimeter']['outer']
        return perimeter_2D(traversing_item_pose, inner_poly, outer_poly, axis=self.axis)


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
    def __init__(self, configs):
        """
        Parameters
        ----------
        configs : list
            List of configuration dictionaries.
        """
        self.configs = configs
        self.constraint_classes = {
            "UprightConstraint": UprightConstraint,
            "HeightConstraint": HeightConstraint,
            "OverUnderConstraint": OverUnderConstraint,
            "Perimeter2DConstraint": Perimeter2DConstraint
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
        for config in self.configs["constraints"]:
            constraints.append(self.classes[config["class"]](**tuple(config["init_args"].values())))
        return constraints
