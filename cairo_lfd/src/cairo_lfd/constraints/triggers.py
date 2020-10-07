"""
The triggers.py module supports methods for identifying triggers for specific events. Generally this is for 
triggering constraints.
"""
from abc import ABCMeta, abstractmethod
import intera_interface
import rospy

from cairo_lfd.services.trigger_service import ConstraintWebTriggerClient


class AbstractTrigger(object):
    """
    Abstract Base class that represents object that can trigger states for Constraints or other objects.
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def check(self):
        """
        Abstract method to get the Item's state.
        """
        pass


class SawyerCuffButtonTrigger(AbstractTrigger):
    """
    Trigger class based on the cuff button presses of a Sawyer.

    Attributes
    ----------
    constraint_id : int
        The id of the constraint for the trigger.
    button : str
        The cuff button name.
    """

    def __init__(self, constraint_id, cuff_button):
        """
        Parameters
        ----------
        constraint_id : int
            The id of the constraint for the trigger.
        cuff_button : str
            The cuff button name.
        """
        self.constraint_id = constraint_id
        self.button = cuff_button
        self.nav = intera_interface.Navigator()

    def check(self):
        """
        Checks whether the button given by button name is currently pressed. State 0 indicates it is not pressed. =

        Returns
        -------
        : int
            Int value indicating if button is pressed.
        """

        if self.nav.get_button_state(self.button) != 0:
            return 1
        else:
            return 0


class WebTrigger(AbstractTrigger):
    """
    Trigger class based a subscribed callback updating the triggered state.
    Currently, this class depends on ConstraintTrigger.msg of the cairo_lfd_msgs package.

    Attributes
    ----------
    constraint_id : int
        The ID of the constraint to check.
    triggered : bool
        The current state of whether the constraint is triggered.
    """

    def __init__(self, constraint_id):
        """
        Parameters
        ----------
        constraint_id : int
            The ID of the constraint to check.
        """
        self.constraint_id = constraint_id
        self.service = ConstraintWebTriggerClient("constraint_trigger_service")
        rospy.loginfo("Web trigger for Constraint ID {} connected to web trigger service.".format(self.constraint_id))

    def check(self):
        """
        Checks the state of the trigger.

        Returns
        -------
        : int
            Int value indicating if triggered state is true or false.
        """
        if self.service.call(self.constraint_id):
            return 1
        else:
            return 0


class TriggerFactory(object):
    """
    Factory class that builds Trigger objects. These items are defined in the config.json file.
    The class field in the configuration determines which constraint class to use.

    Attributes
    ----------
    configs : list
            List of configuration dictionaries.
    classes : dict
        Dictionary with values as uninitialized class references to Trigger class.

    Example
    -------

    Example entry in config.json:

    .. code-block:: json

        {
            "class": "SawyerCuffButtonTrigger",
            "init_args" :
                {
                    "constraint_id": 1,
                    "button": "right_button_square"
                }
        },
        {
            "class": "WebTrigger",
            "init_args":
                {
                    "constraint_id": 1
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
        self.classes = {
            "SawyerCuffButtonTrigger": SawyerCuffButtonTrigger,
            "WebTrigger": WebTrigger
        }

    def generate_triggers(self):
        """
        Build the trigger objects defined in the configuration dictionaries of self.configs.

        Returns
        -------
        robots : list
            List of trigger objects.
        """
        target_constraint_ids = []
        triggers = []
        for config in self.configs:
            if config["init_args"]["constraint_id"] in target_constraint_ids:
                rospy.logwarn("More than one trigger targets the same constraint. Is this intended?")
            target_constraint_ids.append(config["init_args"]["constraint_id"])
            triggers.append(self.classes[config["class"]](**config["init_args"]))
        return triggers
