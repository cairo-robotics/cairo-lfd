import json

import rospy
from std_msgs.msg import String

from cairo_lfd_msgs.srv import ModelUpdateRequest


class ModelUpdateService():
    """

    Attributes
    ----------

    """
    def __init__(self, LFD):
        """
        Parameters
        ----------

        """
        self.command = ""
        self.LFD = LFD
        self.update_server = rospy.Service('/cairo_lfd/model_update', String, self.update_callback)
        self.command_subscriber = rospy.Subscriber('/cairo_lfd/model_commands', String, self.command_callback)
        self.representation_publisher = rospy.Publisher('/cairo_lfd/lfd_representation', String, queue_size=10)

    def command_callback(self, msg):
        self.command = msg.data

    def update_callback(self, req):
        print(req)
        # This request will include the constraint assignment update and eventually parameterizations.

    def start_server(self):
        while self.command != "quit" and not rospy.is_shutdown():
            if self.command == "resample":
                self._clear_command()
                self.LFD.sample_keyframes()
            if self.command == "get_representation":
                self._clear_command()
                data = self.LFD.get_representation()
                self.representation_publisher.publish(json.dumps(data))
            if self.command == "execute":
                self._clear_command()
                self.LFD.perform_skill()

    def _clear_command()
        self.command = ""
