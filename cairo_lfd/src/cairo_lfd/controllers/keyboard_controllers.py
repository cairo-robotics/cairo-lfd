import os

import rospy

from std_msgs.msg import String


class RecordingKeyboardController(object):

    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cairo_lfd/record_command', String, queue_size=10)

    def run_loop(self):
        """
        Return whether or not recording is done.

        Returns
        : bool
            The _done attribute.
        """
        while not rospy.is_shutdown():
            _ = os.system('clear')
            self.print_instructions()
            user_input = raw_input("Enter a command: ")
            if user_input == "":
                self.cmd_pub.publish("")
            elif user_input == "r":
                self.cmd_pub.publish("record")
            elif user_input == "q":
                self.cmd_pub.publish("quit")
            elif user_input == "d":
                self.cmd_pub.publish("discard")
            elif user_input == "c":
                self.cmd_pub.publish("capture")
            elif user_input == "s":
                self.cmd_pub.publish("start")

    def print_instructions(self):
        print("""
        Keyboard interface for collecting demonstrations:
              'r' - Record
              'q' - Quit
              'd' - Discard current demo while recording.
              'c' - Capture current demo while recording.
              's' - Move to start configuration
              """)


class ModelKeyboardController(object):

    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cairo_lfd/model_command', String, queue_size=10)

    def run_loop(self):
        """
        Return whether or not recording is done.

        Returns
        : bool
            The _done attribute.
        """
        while not rospy.is_shutdown():
            _ = os.system('clear')
            self.print_instructions()
            user_input = raw_input("Enter a command: ")
            if user_input == "":
                self.cmd_pub.publish("")
            elif user_input == "e":
                self.cmd_pub.publish("execute")
            elif user_input == "t":
                self.cmd_pub.publish("train")
            elif user_input == "d":
                self.cmd_pub.publish("discard")

    def print_instructions(self):
        print("""
        Keyboard interface for executing and retraining model:
              'e' - Execute skill
              't' - Train models.
              'r' - Resend model representation.
              'q' - Quit
              """)
