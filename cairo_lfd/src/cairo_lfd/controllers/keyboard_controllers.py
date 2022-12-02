import os

import rospy

from std_msgs.msg import String


class RecordingKeyboardController(object):

    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cairo_lfd/record_commands', String, queue_size=10)

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
            user_input = input("Enter a command: ")
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
            elif user_input == "p":
                self.cmd_pub.publish("replay")
            elif user_input == "e":
                self.cmd_pub.publish("end")
            elif user_input == "s":
                self.cmd_pub.publish("start")

    def print_instructions(self):
        print("""
        Keyboard interface for collecting demonstrations:
              'r' - Record
              'q' - Quit Recording Session
              'd' - Discard current demo while recording.
              'c' - Capture current demo/point while recording.
              'p' - Previews/replays demonstration.
              'e' - End the current demo recording. This will capture the entire demonstration if recording a whole demo at once.
              's' - Move to start configuration
              """)


class ModelKeyboardController(object):

    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cairo_lfd/model_commands', String, queue_size=10)

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
            user_input = input("Enter a command: ")
            if user_input == "":
                self.cmd_pub.publish("")
            elif user_input == "execute":
                self.cmd_pub.publish("execute")
            elif user_input == "resample":
                self.cmd_pub.publish("resample")
            elif user_input == "train":
                self.cmd_pub.publish("train")
            elif user_input == "start":
                self.cmd_pub.publish("start")
            elif user_input == "calibrate":
                self.cmd_pub.publish("calibrate")
            elif user_input == "record":
                self.cmd_pub.publish("record")
            elif user_input == "representation":
                self.cmd_pub.publish("get_representation")
            elif user_input == "save":
                self.cmd_pub.publish("save")
            elif user_input == "serialize":
                self.cmd_pub.publish("serialize")
            elif user_input == "quit":
                self.cmd_pub.publish("quit")

    def print_instructions(self):
        print("""
        Keyboard interface for executing and retraining model:
              'execute' - Execute skill
              'resample' - Resample keyframes.
              'representation' - Publish model representation.
              'record' - Record
              'start' - Move to start configuration.
              'calibrate' - Move to calibration pose.
              'train' - Train model with current demonstrations.
              'save' - Save the current subjects demonstrations, raw and labeled.
              'serialize' - Serialize the lfd model.
              'quit' - Quit
              """)
