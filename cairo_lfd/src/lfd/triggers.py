import intera_interface
import rospy


class AbstractSubscribableTrigger(object):
    """
    Abstract Base class for represent items in an Environment.
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def callback(self):
        """
        Abstract method passed to a subscriber as the callback method. This method will be bound to an instance of his class.
        """
        pass

    @abstractmethod
    def trigger(self):
        """
        Abstract method to get the Item's state.
        """
        pass


class AbstractTrigger(object):
    """
    Abstract Base class for represent items in an Environment.
    """
    __metaclass__ = ABCMeta


    @abstractmethod
    def trigger(self):
        """
        Abstract method to get the Item's state.
        """
        pass


class SawyerCuffButtonTrigger(AbstractSubscribableTrigger):

    def __init__(self, cuff_button):
        self.button = cuff_button

    def trigger(self):
        if intera_interface.Navigator().get_button_state(self.button) != 0:
            return 1
        else:
            return 0


class SubscribedTrigger(AbstractTrigger):

    def __init__(self, constraint_name):
        self.constraint_name = constraint_name
        self.triggered = False

    def callback(self, data):
        if data.name == constraint_name:
            if data.on == True:
                self.triggered = True
            else:
                self.triggered = False
        else:
            self.triggered = False

    def trigger(self):
        if self.triggered:
            return 1
        else:
            return 0
