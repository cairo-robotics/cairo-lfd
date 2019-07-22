"""
The triggers.py module supports methods for identifying triggers for specific events. Generally this is for 
triggering constraints.
"""
from abc import ABCMeta, abstractmethod
import intera_interface
import rospy


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


class SubscribedTrigger(AbstractTrigger):
    """
    Trigger class based a subscribed callback updating the triggered state.
    Currently, this class depends on ConstraintTrigger.msg of the cairo_lfd_msgs package.

    Attributes
    ----------
    constraint_name : str
        The name of the constraint to check the parsed data.
    triggered : bool
        The current state of whether the constraint is triggered.
    """

    def __init__(self, constraint_name):
        """
        Parameters
        ----------
        constraint_name : str
            The name of the constraint to check the parsed data.
        """
        self.constraint_name = constraint_name
        self.triggered = False

    def callback(self, data):
        """
        The callback used for a subscriber to check triggered state for the chosen constraint.
        """
        if data.name == constraint_name:
            if data.on is True:
                self.triggered = True
            else:
                self.triggered = False
        else:
            self.triggered = False

    def check(self):
        """
        Checks the state of the trigger.

        Returns
        -------
        : int
            Int value indicating if triggered state is true or false.
        """
        if self.triggered:
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
            "SubscribedTrigger": SubscribedTrigger
        }

    def generate_triggers(self):
        """
        Build the trigger objects defined in the configuration dictionaries of self.configs.

        Returns
        -------
        robots : list
            List of trigger objects.
        """
        triggers = []
        for config in self.configs["triggers"]:
            triggers.append(self.classes[config["class"]](**config["init_args"]))
        return triggers
