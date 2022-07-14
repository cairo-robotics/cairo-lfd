"""
The record.py module contains classes and methods for recording data during live demonstrations.
"""
import sys
import os

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray, String, Float64MultiArray, Bool
from geometry_msgs.msg import PoseStamped, Pose

import intera_interface
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import InteractionOptions, InteractionPublisher

from cairo_lfd.core.environment import Observation, Demonstration
from cairo_lfd.data.labeling import ConstraintKeyframeLabeler
from cairo_lfd.modeling.analysis import evaluate_applied_constraints, check_constraint_validity

from robot_interface.moveit_interface import SawyerMoveitInterface

from collision_ik.srv import CollisionIKSolution, CollisionIKSolutionRequest


import copy


class SawyerDemonstrationRecorder():
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
    def __init__(self, calibration_settings, recording_settings, environment, processor_pipeline=None, publish_constraint_validity=True):
        """
        Parameters
        ----------
        rate : int
            The Hz rate at which to capture state data.
        """
        self.start_configuration = calibration_settings.get("start_configuration", None)
        self._raw_rate = recording_settings.get("sampling_rate", 25)
        self._rate = rospy.Rate(self._raw_rate)
        self._start_time = rospy.get_time()
        self._done = False
        self.publish_constraint_validity = publish_constraint_validity
        self.constraint_trigger = recording_settings.get("constraint_trigger_mechanism", "web_trigger")
        self.processor_pipeline = processor_pipeline
        self.head_display_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
        self.recording_image_path = os.path.join(os.path.dirname(__file__), '../../../../lfd_experiments/images/Recording.jpg')
        self.ready_to_record_image_path = os.path.join(os.path.dirname(__file__), '../../../../lfd_experiments/images/ReadyToRecord.jpg')
        self.command = ""
        self.command_sub = rospy.Subscriber("/cairo_lfd/record_commands", String, self.command_callback)
        self.environment = environment
        self._setup_zero_g()

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

    def start(self):
        self._done = False

    def record(self):
        """


        Returns
        -------
        demonstrations : list
            List of Demonstration objects each of which captures Observations during user demonstrations.
        """

        demonstrations = []
        rospy.loginfo("Ready to record. Sampling will be at ~{}Hz".format(self._raw_rate))
        self.start()
        while not self.done():
            self.head_display_pub.publish(self._setup_image(self.ready_to_record_image_path))
            if self.command == "record":
                print("Recording!")
                self.head_display_pub.publish(self._setup_image(self.recording_image_path))
                self.interaction_publisher.external_rate_send_command(self.interaction_options)
                observations = []
                counter = 0
                observations = self._record_demonstration()
                if len(observations) > 0:
                    demonstrations.append(Demonstration(observations))
            if self.command == "discard":
                if len(demonstrations) > 0:
                    demonstrations.pop()
                    print("Discarded most recent demonstration recording!")
                else:
                    print("There are no new demonstrations to discard.")
                self._clear_command()
            if self.command == "quit":
                print("Stopping recording!")
                self._clear_command()
                self.stop()
            if self.command == "start":
                print("Moving to start position!")
                self._move_to_start_configuration()

        if self.processor_pipeline is not None:
            # Generates additional data
            self.processor_pipeline.process(demonstrations)
        self._analyze_applied_constraints(self.environment, demonstrations)
        self._clear_command()
        return demonstrations

    def _move_to_start_configuration(self):
        """ Create the moveit_interface """
        if self.start_configuration is not None:
            moveit_interface = SawyerMoveitInterface()
            moveit_interface.set_velocity_scaling(.35)
            moveit_interface.set_acceleration_scaling(.25)
            moveit_interface.set_joint_target(self.start_configuration)
            moveit_interface.execute(moveit_interface.plan())
        else:
            rospy.logwarn("No start configuration provided in your config.json file.")
        self._clear_command()

    def _record_demonstration(self):
        observations = []
        while True:
            robot = self.environment.robot
            if robot._gripper:
                if robot._cuff.upper_button():
                    robot._gripper.open()
                elif robot._cuff.lower_button():
                    robot._gripper.close()
            data = {
                "time": self._time_stamp(),
                "robot": self.environment.get_robot_state(),
                "items": self.environment.get_item_state(),
                "triggered_constraints": self.environment.check_constraint_triggers()
            }
            observation = Observation(data)
            if self.publish_constraint_validity:
                valid_constraints = check_constraint_validity(self.environment, self.environment.constraints, observation)[1]
                pub = rospy.Publisher('cairo_lfd/valid_constraints', Int8MultiArray, queue_size=10)
                msg = Int8MultiArray(data=valid_constraints)
                pub.publish(msg)
            observations.append(observation)
            if self.command == "discard":
                rospy.loginfo("~~~DISCARDED DEMONSTRATION~~~")
                self.interaction_publisher.send_position_mode_cmd()
                self._clear_command()
                return []
            if self.command == "capture":
                rospy.loginfo("~~~CAPTURED DEMONSTRATION~~~")
                self.interaction_publisher.send_position_mode_cmd()
                self._clear_command()
                rospy.loginfo("{} observations captured.".format(len(observations)))
                return observations
            if self.command == "end":
                rospy.loginfo("~~~CAPTURED DEMONSTRATION~~~")
                self.interaction_publisher.send_position_mode_cmd()
                self._clear_command()
                return observations
            self._rate.sleep()

    def _analyze_applied_constraints(self, environment, demos):
        # Analyze observations for constraints. If using web triggered constraints, we don't evaluate and
        # instead the applied constraints are those explicitly set by the user.
        for demo in demos:
            if self.constraint_trigger == 'cuff_trigger':
                # Using the cuff trigger will cause a propagation forward of current set of applied constraints
                evaluate_applied_constraints(environment, demo.observations)
            elif self.constraint_trigger == 'web_trigger':
                for observation in demo.observations:
                    observation.data["applied_constraints"] = observation.get_triggered_constraint_data()
            else:
                rospy.logerr("No valid constraint trigger mechanism passed.")

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

    def _setup_zero_g(self):
        ########################################
        #  Setup Intera to Support Zero-G Mode #
        ########################################
        interaction_pub = InteractionPublisher()
        interaction_options = InteractionOptions()
        interaction_options.set_max_impedance([False])
        interaction_options.set_rotations_for_constrained_zeroG(True)
        interaction_frame = Pose()
        interaction_frame.position.x = 0
        interaction_frame.position.y = 0
        interaction_frame.position.z = 0
        interaction_frame.orientation.x = 0
        interaction_frame.orientation.y = 0
        interaction_frame.orientation.z = 0
        interaction_frame.orientation.w = 1
        interaction_options.set_K_impedance([0, 0, 0, 0, 0, 0])
        interaction_options.set_K_nullspace([0, 0, 0, 0, 0, 0, 0])
        interaction_options.set_interaction_frame(interaction_frame)
        rospy.loginfo(interaction_options.to_msg())
        rospy.on_shutdown(interaction_pub.send_position_mode_cmd)
        self.interaction_publisher = interaction_pub
        self.interaction_options = interaction_options

    def _clear_command(self):
        self.command = ""

class SawyerPointwiseRecorder():
    """
    Class to record state data from the ReThink Robotics Sawyer robot for capturing demonstrated data for 
    LfD experimentation one single point at a time.

    Attributes
    ----------

    _start_time : float
        Current time in ROScore.
    _done : bool
        Termination flag.
    """
    def __init__(self, settings, environment, processor_pipeline=None, publish_constraint_validity=True):
        """
        Parameters
        ----------
        rate : int
            The Hz rate at which to capture state data.
        """
        self.start_configuration = settings.get("start_configuration", None)
        self._start_time = rospy.get_time()
        self._raw_rate = settings.get("sampling_rate", 25)
        self._rate = rospy.Rate(self._raw_rate)
        self._done = False
        self.publish_constraint_validity = publish_constraint_validity
        self.constraint_trigger = settings.get("constraint_trigger_mechanism", "web_trigger")
        self.processor_pipeline = processor_pipeline
        self.head_display_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
        self.recording_image_path = os.path.join(os.path.dirname(__file__), '../../../../lfd_experiments/images/Recording.jpg')
        self.ready_to_record_image_path = os.path.join(os.path.dirname(__file__), '../../../../lfd_experiments/images/ReadyToRecord.jpg')
        self.command = ""
        self.command_sub = rospy.Subscriber("/cairo_lfd/record_commands", String, self.command_callback)
        self.environment = environment
        self._setup_zero_g()

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

    def start(self):
        self._done = False

    def record(self):
        """


        Returns
        -------
        demonstrations : list
            List of Demonstration objects each of which captures Observations during user demonstrations.
        """

        demonstrations = []
        rospy.loginfo("Ready to record single points. Refresh rate will be at ~{}Hz".format(self._raw_rate))
        self.start()
        while not self.done():
            self.head_display_pub.publish(self._setup_image(self.ready_to_record_image_path))
            if self.command == "record":
                print("Recording!")
                self.head_display_pub.publish(self._setup_image(self.recording_image_path))
                self.interaction_publisher.external_rate_send_command(self.interaction_options)
                observations = []
                counter = 0
                observations = self._record_demonstration()
                if len(observations) > 0:
                    demonstrations.append(Demonstration(observations))
            if self.command == "discard":
                if len(demonstrations) > 0:
                    demonstrations.pop()
                    print("Discarded most recent demonstration recording!")
                else:
                    print("There are no new demonstrations to discard.")
                self._clear_command()
            if self.command == "quit":
                print("Stopping recording!")
                self._clear_command()
                self.stop()
            if self.command == "start":
                print("Moving to start position!")
                self._move_to_start_configuration()

        if self.processor_pipeline is not None:
            # Generates additional data
            self.processor_pipeline.process(demonstrations)
        self._analyze_applied_constraints(self.environment, demonstrations)
        self._clear_command()
        return demonstrations

    def _move_to_start_configuration(self):
        """ Create the moveit_interface """
        if self.start_configuration is not None:
            moveit_interface = SawyerMoveitInterface()
            moveit_interface.set_velocity_scaling(.35)
            moveit_interface.set_acceleration_scaling(.25)
            moveit_interface.set_joint_target(self.start_configuration)
            moveit_interface.execute(moveit_interface.plan())
        else:
            rospy.logwarn("No start configuration provided in your config.json file.")
        self._clear_command()

    def _record_demonstration(self):
        observations = []
        while True:
            robot = self.environment.robot
            if robot._gripper:
                if robot._cuff.upper_button():
                    robot._gripper.open()
                elif robot._cuff.lower_button():
                    robot._gripper.close()
            data = {
                "time": self._time_stamp(),
                "robot": self.environment.get_robot_state(),
                "items": self.environment.get_item_state(),
                "triggered_constraints": self.environment.check_constraint_triggers()
            }
            observation = Observation(data)
            if self.publish_constraint_validity:
                valid_constraints = check_constraint_validity(self.environment, self.environment.constraints, observation)[1]
                pub = rospy.Publisher('cairo_lfd/valid_constraints', Int8MultiArray, queue_size=10)
                msg = Int8MultiArray(data=valid_constraints)
                pub.publish(msg)
            if self.command == "discard":
                rospy.loginfo("~~~DISCARDED~~~")
                self.interaction_publisher.send_position_mode_cmd()
                self._clear_command()
                return []
            if self.command == "capture":
                rospy.loginfo("~~~CAPTURED POINT~~~")
                self._clear_command()
                rospy.loginfo("{} observations captured.".format(len(observations)))
                observations.append(observation)
            if self.command == "end":
                rospy.loginfo("~~~ENDING POINTWISE RECORDING~~~")
                self.interaction_publisher.send_position_mode_cmd()
                self._clear_command()
                return observations
            self._rate.sleep()

    def _analyze_applied_constraints(self, environment, demos):
        # Analyze observations for constraints. If using web triggered constraints, we don't evaluate and
        # instead the applied constraints are those explicitly set by the user.
        for demo in demos:
            if self.constraint_trigger == 'cuff_trigger':
                # Using the cuff trigger will cause a propagation forward of current set of applied constraints
                evaluate_applied_constraints(environment, demo.observations)
            elif self.constraint_trigger == 'web_trigger':
                for observation in demo.observations:
                    observation.data["applied_constraints"] = observation.get_triggered_constraint_data()
            else:
                rospy.logerr("No valid constraint trigger mechanism passed.")

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

    def _setup_zero_g(self):
        ########################################
        #  Setup Intera to Support Zero-G Mode #
        ########################################
        interaction_pub = InteractionPublisher()
        interaction_options = InteractionOptions()
        interaction_options.set_max_impedance([False])
        interaction_options.set_rotations_for_constrained_zeroG(True)
        interaction_frame = Pose()
        interaction_frame.position.x = 0
        interaction_frame.position.y = 0
        interaction_frame.position.z = 0
        interaction_frame.orientation.x = 0
        interaction_frame.orientation.y = 0
        interaction_frame.orientation.z = 0
        interaction_frame.orientation.w = 1
        interaction_options.set_K_impedance([0, 0, 0, 0, 0, 0])
        interaction_options.set_K_nullspace([0, 0, 0, 0, 0, 0, 0])
        interaction_options.set_interaction_frame(interaction_frame)
        rospy.loginfo(interaction_options.to_msg())
        rospy.on_shutdown(interaction_pub.send_position_mode_cmd)
        self.interaction_publisher = interaction_pub
        self.interaction_options = interaction_options

    def _clear_command(self):
        self.command = ""

class ARPOLfDRecorder():
    
    def __init__(self, calibration_settings, recording_settings, environment, processor_pipeline=None, publish_constraint_validity=True):
        """
        Parameters
        ----------
        rate : int
            The Hz rate at which to capture state data.
        """
        self.start_configuration = calibration_settings.get("start_configuration", None)
        self._start_time = rospy.get_time()
        self._raw_rate = recording_settings.get("sampling_rate", 25)
        self._rate = rospy.Rate(self._raw_rate)
        self._done = False
        self.processor_pipeline = processor_pipeline
        self.joint_angle_publisher = rospy.Publisher('arpo_lfd/joint_angles', Float64MultiArray, queue_size=10)
        self.clear_ar_traj_publisher = rospy.Publisher('arpo_lfd/clear_traj_cmd', Bool, queue_size=10)
        self.head_display_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)
        self.recording_image_path = os.path.join(os.path.dirname(__file__), '../../../../lfd_experiments/images/Recording.jpg')
        self.ready_to_record_image_path = os.path.join(os.path.dirname(__file__), '../../../../lfd_experiments/images/ReadyToRecord.jpg')
        self.command = ""
        self.command_sub = rospy.Subscriber("/cairo_lfd/record_commands", String, self.command_callback)
        self.environment = environment

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

    def start(self):
        self._done = False

    def record(self):

        demonstrations = []
        rospy.loginfo("Ready to record single points. Refresh rate will be at ~{}Hz".format(self._raw_rate))
        self.start()
        while not self.done():
            if self.command == "record":
                print("Recording!")
                self.clear_ar_traj_publisher.publish(True)
    
                observations = []
                counter = 0
                observations = self._record_demonstration()
                if len(observations) > 0:
                    demonstrations.append(Demonstration(observations))
            if self.command == "discard":
                if len(demonstrations) > 0:
                    demonstrations.pop()
                    self.clear_ar_traj_publisher.publish(True)
                    print("Discarded most recent demonstration recording!")
                else:
                    print("There are no new demonstrations to discard.")
                self._clear_command()
            if self.command == "quit":
                print("Stopping recording!")
                self._clear_command()
                self.stop()
            if self.command == "start":
                print("Moving to start position!")
                self._move_to_start_configuration()

        if self.processor_pipeline is not None:
            # Generates additional data
            self.processor_pipeline.process(demonstrations)
        self._analyze_applied_constraints(self.environment, demonstrations)
        self._clear_command()
        return demonstrations

    def _move_to_start_configuration(self):
        """ Create the moveit_interface """
        if self.start_configuration is not None:
            moveit_interface = SawyerMoveitInterface()
            moveit_interface.set_velocity_scaling(.35)
            moveit_interface.set_acceleration_scaling(.25)
            moveit_interface.set_joint_target(self.start_configuration)
            moveit_interface.execute(moveit_interface.plan())
        else:
            rospy.logwarn("No start configuration provided in your config.json file.")
        self._clear_command()

    def _record_demonstration(self):
        observations = []
       
        while True:
            data = {
                "time": self._time_stamp(),
                "robot": self.environment.get_robot_state(),
                "items": self.environment.get_item_state(),
                "triggered_constraints": self.environment.check_constraint_triggers()
            }
            print(data["robot"])
            message = Float64MultiArray()
            message.data = data["robot"]["joint_angle"]
            self.joint_angle_publisher.publish(message)
                
            observation = Observation(data)
            # if self.publish_constraint_validity:
            #     valid_constraints = check_constraint_validity(self.environment, self.environment.constraints, observation)[1]
            #     pub = rospy.Publisher('cairo_lfd/valid_constraints', Int8MultiArray, queue_size=10)
            #     msg = Int8MultiArray(data=valid_constraints)
            #     pub.publish(msg)
            if self.command == "discard":
                rospy.loginfo("~~~DISCARDED~~~")
                self.interaction_publisher.send_position_mode_cmd()
                self.clear_ar_traj_publisher.publish(True)
                self._clear_command()
                return []
            if self.command == "capture":
                rospy.loginfo("~~~CAPTURED POINT~~~")
                self._clear_command()
                rospy.loginfo("{} observations captured.".format(len(observations)))
                observations.append(observation)
            if self.command == "end":
                rospy.loginfo("~~~ENDING POINTWISE RECORDING~~~")
                self.interaction_publisher.send_position_mode_cmd()
                self._clear_command()
                return observations
            self._rate.sleep()
    
    def _joint_configuration_cb(self, msg):
        self.current_joint_configuration = msg.data