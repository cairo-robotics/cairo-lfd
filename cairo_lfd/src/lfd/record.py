"""
The record.py module contains classes and methods for recording data during live demonstrations.
"""
import rospy
from lfd.environment import Observation, Demonstration
import sys
import select
import pudb
import os
import intera_interface
import cv2
import cv_bridge
from sensor_msgs.msg import Image


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
    def __init__(self, rate, interaction_publisher=None, interaction_options=None):
        """
        Parameters
        ----------
        rate : int
            The rate at which to capture state data.
        """
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False
        self.interaction_publisher = interaction_publisher
        self.interaction_options = interaction_options
        self.head_display_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
        self.recording_image_path = os.path.join(os.path.dirname(__file__), '../../../lfd_experiments/images/Recording.jpg')
        self.ready_to_record_image_path = os.path.join(os.path.dirname(__file__), '../../../lfd_experiments/images/ReadyToRecord.jpg')

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

    def record_demonstrations(self, environment, auto_zeroG=False):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.

        Parameters
        ----------
        environment: Environment
            The Environment object of the current LfD experiment.

        Returns
        -------
        demosntrations : list
            List of Demonstration objects each of which captures Observations during user demonstrations.
        """
        robot = environment.robot

        demonstrations = []

        while not self.done():
            print("Press 'r' or hold 'ii' cuff button to enter recording mode or 'q' to quit.\n")
            recording = False
            user_input = ''
            while user_input != 'q':
                self.head_display_pub.publish(self._setup_image(self.ready_to_record_image_path))
                # pu.db
                stdin, stdout, stderr = select.select([sys.stdin], [], [], .0001)
                for s in stdin:
                    if s == sys.stdin:
                        user_input = sys.stdin.readline().strip()
                if environment.robot._navigator.get_button_state("right_button_show") == 2 or user_input == "r":
                    print("Recording!")
                    recording = True
                observations = []
                while recording:
                    self.head_display_pub.publish(self._setup_image(self.recording_image_path))
                    if auto_zeroG and self.interaction_publisher is not None and self.interaction_options is not None:
                        self.interaction_publisher.external_rate_send_command(self.interaction_options)
                    elif auto_zeroG and self.interaction_publisher is None or self.interaction_options is None:
                        rospy.logerr("Sawyer Recorder must possess an interaction publisher and/or interaction options for zeroG")
                        raise ValueError("Sawyer Recorder must possess an interaction publisher and/or interaction options zeroG")
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
                    observations.append(observation)
                    stdin, stdout, stderr = select.select([sys.stdin], [], [], .0001)
                    for s in stdin:
                        if s == sys.stdin:
                            user_input = sys.stdin.readline().strip()
                    if user_input == "d":
                        rospy.loginfo("~~~DISCARDED~~~")
                        self.interaction_publisher.send_position_mode_cmd()
                        user_input = ''
                        recording = False
                        print("Demonstration discarded!\n Press 'r' or hold center cuff wheel to record again or 'q' to end the session.\n")
                    if environment.robot._navigator.get_button_state("right_button_back") == 2 or user_input == "c":
                        demonstrations.append(Demonstration(observations))
                        rospy.loginfo("~~~CAPTURED~~~")
                        self.interaction_publisher.send_position_mode_cmd()
                        user_input = ''
                        recording = False
                        print("Demonstration captured!\n Press 'r' or hold center cuff wheel to record again or 'q' to end the session.\n")
                    self._rate.sleep()
            if user_input == 'q':
                self.stop()

        return demonstrations

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