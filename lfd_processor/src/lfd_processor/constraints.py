import intera_interface
from predicate_classification.predicate_classifiers import height, upright
from lfd_processor.processing import convert_data_to_pose


class HeightConstraint(object):

    """
    HeightConstraint class to evaluate the height predicate classifier assigned to a given item. 

    height() returns true if object distance from reference_height is greater than threshold distnace.
        A positive threshold distance mean height above
        A negative threshold distance means height below.
    """

    def __init__(self, constraint_id, item_id, button, reference_height, threshold_distance):

        """
        These arguemnts should be in the "init_args" field of the config.json file's entry representing 
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
        betweeen Sawyer (end-effector) and general items (blocks etc,.). 

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
    """

    def __init__(self, constraint_id, item_id, button, threshold_angle, axis):

        """
        These arguemnts should be in the "init_args" field of the config.json file's entry representing this constraint.

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
        self.button = button
        self.threshold_angle = threshold_angle
        self.axis = str(axis)

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
        betweeen Sawyer (end-effector) and general items (blocks etc,.). 

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
