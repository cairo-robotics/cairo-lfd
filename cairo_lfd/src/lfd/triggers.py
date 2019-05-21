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
    button : str
        The cuff button name.
    """

    def __init__(self, cuff_button):
        """
        Parameters
        ----------
        cuff_button : str
            The cuff button name.
        """
        self.button = cuff_button

    def check(self):
        """
        Checks whether the button given by button name is currently pressed. State 0 indicates it is not pressed. =

        Returns
        -------
        : int
            Int value indicating if button is pressed.
        """
        if intera_interface.Navigator().get_button_state(self.button) != 0:
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
