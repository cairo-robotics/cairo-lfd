import rospy

from std_msgs.msg import String

import intera_interface


class RecordingCuffController(object):

    def __init__(self):
        self._navigator = intera_interface.Navigator()
        self.cmd_pub = rospy.Publisher('/cairo_lfd/record_commands', String, queue_size=10)

    def run_loop(self):
        """
        Return whether or not recording is done.

        Returns
        : bool
            The _done attribute.
        """
        self.print_instructions()
        while not rospy.is_shutdown():
            if self._navigator.get_button_state("right_button_show") == 2:
                self.cmd_pub.publish("record")
            elif self._navigator.get_button_state("right_button_back") == 2:
                self.cmd_pub.publish("quit")
            elif self._navigator.get_button_state("right_button_back") == 3:
                self.cmd_pub.publish("discard")
            elif self._navigator.get_button_state("right_button_show") == 3:
                self.cmd_pub.publish("capture")
            elif self._navigator.get_button_state("right_button_ok") == 2:
                self.cmd_pub.publish("start")

    def print_instructions(self):
        print("""
        THIS IS NOT CONFIGURED TO WORK WITH POINTWISE RECORDING YET.
        Sawyer Cuff Buttons interface for collecting demonstrations:
              hold 'ii' - Record
              hold '<--' - Quit 
              double tap '<--' - Discard current demo while recording.
              double tap 'ii' - Capture current demo while recording.
              press & hold 'wheel' - Move to start configuration
              """)
