"""
The record.py module contains classes and methods for recording data during live demonstrations.
"""
import sys
import os

import rospy
import intera_interface
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, String

from cairo_lfd.core.environment import Observation, Demonstration
from robot_interface.moveit_interface import SawyerMoveitInterface


class KeyboardController(object):

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
                self.cmd_pub.publish("reset")


class SawyerRecorder(object):
    """
    Class to record state data from the ReThink Robotics Sawyer robot for capturing demonstrated data for 
    LfD experimentation.

    Attributes
    ----------
    _raw_rate : int
        The rate at which to capture state data.
    _rate : int
        The ROScore rate.
    _start_time : float
        Current time in ROScore.
    _done : bool
        Termination flag.
    """
    def __init__(self, reset_pose, rate, interaction_publisher=None, interaction_options=None):
        """
        Parameters
        ----------
        rate : int
            The rate at which to capture state data.
        """
        self._reset_pose = reset_pose
        self._raw_rate = rate
        self._rate = rospy.Rate(self._raw_rate)
        self._start_time = rospy.get_time()
        self._done = False
        self.interaction_publisher = interaction_publisher
        self.interaction_options = interaction_options
        self.head_display_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
        self.recording_image_path = os.path.join(os.path.dirname(__file__), '../../../../lfd_experiments/images/Recording.jpg')
        self.ready_to_record_image_path = os.path.join(os.path.dirname(__file__), '../../../../lfd_experiments/images/ReadyToRecord.jpg')
        self.command = ""
        self.command_sub = rospy.Subscriber("/cairo_lfd/record_command", String, self.command_callback)

    def command_callback(self, data):
        self.command = data.data

    def _time_stamp(self):
        """
        Captures time difference from start time to current time.

        Returns
        ----------
         : float
            Current time passed so far.
        """
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording by setting _done flag to True.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.

        Returns
        : bool
            The _done attribute.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def run(self, environment, constraint_analyzer=None, auto_zeroG=False):
        """

        Parameters
        ----------
        environment: Environment
            The Environment object of the current LfD experiment.

        Returns
        -------
        demonstrations : list
            List of Demonstration objects each of which captures Observations during user demonstrations.
        """

        demonstrations = []

        while not self.done():
            self.head_display_pub.publish(self._setup_image(self.ready_to_record_image_path))
            if self.command == "record":
                print("Recording!")
                self.head_display_pub.publish(self._setup_image(self.recording_image_path))
                if auto_zeroG and self.interaction_publisher is not None and self.interaction_options is not None:
                    self.interaction_publisher.external_rate_send_command(self.interaction_options)
                elif auto_zeroG and self.interaction_publisher is None or self.interaction_options is None:
                    rospy.logerr("Sawyer Recorder must possess an interaction publisher and/or interaction options for zeroG")
                    raise ValueError("Sawyer Recorder must possess an interaction publisher and/or interaction options zeroG")
                observations = []
                counter = 0
                observations = self._record_demonstration(environment, constraint_analyzer=None)
                if len(observations) > 0:
                    demonstrations.append(Demonstration(observations))
            if self.command == "quit":
                print("Quitting!")
                self._clear_command()
                self.stop()
            if self.command == "reset":
                print("Moving to reset position!")
                self._move_to_reset_pose()
        return demonstrations

    def _move_to_reset_pose(self):
        """ Create the moveit_interface """
        moveit_interface = SawyerMoveitInterface()
        moveit_interface.set_velocity_scaling(.35)
        moveit_interface.set_acceleration_scaling(.25)
        moveit_interface.set_pose_target(self._reset_pose)
        moveit_interface.execute(moveit_interface.plan())
        self._clear_command()

    def _record_demonstration(self, environment, constraint_analyzer):
        observations = []
        while True:
            robot = environment.robot
            if robot._gripper:
                if robot._cuff.upper_button():
                    robot._gripper.open()
                elif robot._cuff.lower_button():
                    robot._gripper.close()
            data = {
                "time": self._time_stamp(),
                "robot": environment.get_robot_state(),
                "items": environment.get_item_state(),
                "triggered_constraints": environment.check_constraint_triggers()
            }
            observation = Observation(data)

            if constraint_analyzer is not None:
                valid_constraints = constraint_analyzer.evaluate(environment.constraints, observation)[1]
                pub = rospy.Publisher('/cairo_lfd/valid_constraints', Int16MultiArray, queue_size=10)
                msg = Int16MultiArray(data=valid_constraints)
                pub.publish(msg)

            observations.append(observation)
            if self.command == "discard":
                rospy.loginfo("~~~DISCARDED~~~")
                self.interaction_publisher.send_position_mode_cmd()
                print("Demonstration discarded!\n Press 'r' or 'ii' cuff button record again or 'q' to end the session.\n")
                self._clear_command()
                return []
            if self.command == "capture":
                rospy.loginfo("~~~CAPTURED~~~")
                self.interaction_publisher.send_position_mode_cmd()
                self._clear_command()
                print("Demonstration captured!\n Press 'r' or 'ii' cuff button record or hold center cuff wheel to record again or 'q' to end the session.\n")
                return observations
            self._rate.sleep()

    def _setup_image(self, image_path):
        """
        Load the image located at the specified path
        @type image_path: str
        @param image_path: the relative or absolute file path to the image file
        @rtype: sensor_msgs/Image or None
        @param: Returns sensor_msgs/Image if image convertable and None otherwise
        """
        if not os.access(image_path, os.R_OK):
            rospy.logerr("Cannot read file at '{0}'".format(image_path))
            return None

        img = cv2.imread(image_path)
        # Return msg
        return cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")

    def _clear_command(self):
        self.command = ""
