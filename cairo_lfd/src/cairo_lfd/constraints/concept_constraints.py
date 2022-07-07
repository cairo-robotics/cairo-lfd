"""
The constraints.py module contains a classes that encapsulate predicate classifiers used to
evaluate binary value conceptual constraints.
"""
import rospy

from predicate_classification.pose_classifiers import planar, twist, over_under, cone
from predicate_classification.path_classifiers import perimeter_2D
from cairo_lfd.data.conversion import convert_data_to_pose


class PlanarConstraint(object):
    """
    PlanarConstraint class to evaluate the planar predicate classifier assigned to a given item.

    planar() returns true if object distance from reference_height is greater than threshold distnace.

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
    def __init__(self, constraint_id, item_id, reference_position, threshold_distance, direction="positive", axis="z"):

        """
        These arguments should be in the "init_args" field of the config.json file's entry representing
        this constraint.

        Parameters
        ----------
        constraint_id : int/tuple
            Id of the constraint as defined in the config.json file. Can be a tuple when used within the context
            of a metaconstraint ('height', 0) for sampling purposes.
        item_id : int
            Id of the item on which the constraint can be applied.
        reference_position : int
            The reference or starting position to compare an objects height distance against the threshold_distance along the given axis.
        threshold_distance : int
            The distance from reference (positive: above; negative; below) to compare an object's distance from reference.
        direction : str
            The direction of relative to the axis to evaluate the constraint. 'positive' indicates positive direction along chosen axis or height above. 'negative' means negative direction along chosen axis.
        axis : str
            Axis to evaluate constraint (x, y, or z)
        """

        self.id = constraint_id
        self.item_id = item_id
        self.reference_position = reference_position
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

        return planar(item_pose, self.reference_position, self.threshold_distance, direction=self.direction, axis=self.axis)

    def __repr__(self):
        return "PlanarConstraint({})".format(self.__dict__)


class ConeConstraint(object):
    """
    The ConeConstraint class evaluates the orientation predicate classifier assigned to a given item.

    orientation() returns true if object's orientation is within a cone centered around a given axis and with a threshold angle the allowed deviations from dead center in that cone. 

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
    def __init__(self, constraint_id, item_id, threshold_angle, reference_orientation=None):
        """
        These arguments should be in the "init_args" field of the config.json file's entry representing this constraint.

        Parameters
        ----------
        constraint_id : int/tuple
            Id of the constraint as defined in the config.json file. Can be a tuple when used within the context
            of a metaconstraint ('orientation', 0) for sampling purposes.
        item_id : int
            Id of the item on which the constraint can be applied.
        threshold_angle : int
            The angle within which the assigned item's (from item_id) current orientation must be compared with its defined upright position.
        reference_orientation : list
            The x, y, z, w values of the orientation that should be used as the upright orientation rather than grabbing orientation assigned to the item.
        """
        self.id = constraint_id
        self.item_id = item_id
        self.threshold_angle = threshold_angle
        self.reference_orientation = reference_orientation

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
        if self.reference_orientation is None:
            upright_pose = convert_data_to_pose(item_info["reference_pose"]["position"], item_info["reference_pose"]["orientation"])
        else:
            upright_pose = convert_data_to_pose([0, 0, 0], self.reference_orientation)
        return cone(upright_pose, current_pose, self.threshold_angle)

    def __repr__(self):
        return "ConeConstraint({}, {}, {}, {}, {})".format(self.id, self.item_id, self.threshold_angle, self.reference_orientation)


class OrientationConstraint(object):
    """
    The OrientationConstraint class evaluates constraint validity using the cone and twist predicate classifiers.

    orientation() returns true if object's orientation is within a cone centered around a given axis and with a 
    threshold angle dead center in that cone as well as ensuring the item is not twisted about that axis by the same
    degree threshold.

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
    def __init__(self, constraint_id, item_id, threshold_angle, axis="z", reference_orientation=None):
        """
        These arguments should be in the "init_args" field of the config.json file's entry representing this constraint.

        Parameters
        ----------
        constraint_id : int/tuple
            Id of the constraint as defined in the config.json file. Can be a tuple when used within the context
            of a metaconstraint ('orientation', 0) for sampling purposes.
        item_id : int
            Id of the item on which the constraint can be applied.
        threshold_angle : int
            The angle within which the assigned item's (from item_id) current orientation must be compared with its
            defined upright position.
        axis : str
            The axis from which angle of deviation is calculated.
        reference_orientation : list
            The x, y, z, w values of the orientation that should be used as the upright orientation rather than grabbing orientation assigned to the item.
        """
        self.id = constraint_id
        self.item_id = item_id
        self.threshold_angle = threshold_angle
        self.axis = axis
        self.reference_orientation = reference_orientation

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
        if self.reference_orientation is None:
            upright_pose = convert_data_to_pose(item_info["upright_pose"]["position"], item_info["upright_pose"]["orientation"])
        else:
            upright_pose = convert_data_to_pose([0, 0, 0], self.reference_orientation)
        cone_eval = cone(upright_pose, current_pose, self.threshold_angle, self.axis)
        twist_eval = twist(upright_pose, current_pose, self.threshold_angle)
        if cone_eval and twist_eval:
            return 1
        else:
            return 0

    def __repr__(self):
        return "OrientationConstraint({}, {}, {}, {}, {})".format(self.id, self.item_id, self.threshold_angle, self.axis, self.reference_orientation)


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
    def __init__(self, constraint_id, above_item_id, below_item_id, threshold_distance, axis, reference_pose=None):

        """
        These arguments should be in the "init_args" field of the config.json file's entry representing
        this constraint.

        Parameters
        ----------
        constraint_id : int/tuple
            Id of the constraint as defined in the config.json file. Can be a tuple when used within the context
            of a metaconstraint ('overunder', 0) for sampling purposes.
        above_item_id : int
            Id of the item that must be above the other for the constraint to hold true.
        below_item_id : int
            Id of the item that must be below the other for the constraint to hold true.
        threshold_distance : int
            The distance from reference (positive: above; negative; below) to compare an object's distance
            from reference.
        axis : str
            The axis from which angle of deviation is calculated.
        reference_pose : dict
            Dictionary of position and orientation.
        """
        self.id = constraint_id
        self.above_item_id = above_item_id
        self.below_item_id = below_item_id
        self.threshold_distance = threshold_distance
        self.axis = str(axis)
        self.reference_pose = reference_pose

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
        if self.reference_pose is None:
            above_pose, below_pose = self._get_poses(environment, observation)
        else:
            rospy.logdebug("Using reference pose")
            above_pose, below_pose = self._get_poses_with_reference(environment, observation)

        return over_under(above_pose, below_pose, self.threshold_distance, axis=self.axis)

    def __repr__(self):
        return "OverUnderConstraint({}, {}, {}, {}, {})".format(self.id, self.above_item_id, self.below_item_id, self.threshold_distance, self.axis)

    def _get_poses(self, environment, observation):
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
        return above_pose, below_pose

    def _get_poses_with_reference(self, environment, observation):
        if self.above_item_id == int(environment.get_robot_info()["id"]):
            above_data = observation.get_robot_data()
            above_pose = convert_data_to_pose(above_data["position"], above_data["orientation"])
            below_pose = convert_data_to_pose(self.reference_pose["position"], self.reference_pose["orientation"])
        else:
            above_data = observation.get_item_data(self.above_item_id)
            above_pose = convert_data_to_pose(above_data["position"], above_data["orientation"])
            below_pose = convert_data_to_pose(self.reference_pose["position"], self.reference_pose["orientation"])
        return above_pose, below_pose


class Perimeter2DConstraint(object):
    """
    Perimeter class to evaluate the perimeter predicate classifier assigned to a given item.

    over_under() returns true if one pose is above another pose and within the a threshold distance
    in the plane orthogonal to the given axis.

    Attributes
    ----------
    id : int / tuple
        Id of the constraint as defined in the config.json file. Can be a tuple when used within the context
            of a metaconstraint ('perimeter', 0) for sampling purposes.
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

    def __repr__(self):
        return "Perimeter2DConstraint({}, {}, {}, {})".format(self.id, self.perimeter_item_id, self.traversing_item_id, self.axis)


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
            "ConeConstraint": ConeConstraint,
            "OrientationConstraint": OrientationConstraint,
            "PlanarConstraint": PlanarConstraint,
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
        constraint_ids = []
        constraints = []
        for config in self.configs:
            if config["init_args"]["constraint_id"] in constraint_ids:
                raise ValueError(
                    "Constraints must each have a unique integer 'constraint_id'")
            else:
                constraint_ids.append(config["init_args"]["constraint_id"])
            try:
                constraints.append(self.constraint_classes[config["class"]](**config["init_args"]))
            except TypeError as e:
                rospy.logerr("Error constructing {}: {}".format(self.constraint_classes[config["class"]], e))
        return constraints
