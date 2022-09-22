import os
import json
import uuid

import rospy
from std_msgs.msg import String

from cairo_lfd.data.io import export_to_json
from cairo_lfd.middleware.ar_middleware import remap_constraints_for_lfd
from cairo_lfd.data.conversion import convert_data_to_pose

from robot_interface.moveit_interface import SawyerMoveitInterface


class ARPOLfDStudyController():
    """

    Attributes
    ----------

    """

    def __init__(self, calibration_settings, lfd_model, recorder, labeler, poor_demonstrations, output_directory, task, subject):
        """
        Parameters
        ----------

        """
        self.command = ""
        self.calibration_pose = calibration_settings.get("calibration_pose", None)
        self.start_configuration = calibration_settings.get("start_configuration", None)
        self.lfd_model = lfd_model
        self.recorder = recorder
        self.labeler = labeler
        self.command_subscriber = rospy.Subscriber(
            '/cairo_lfd/model_commands', String, self._command_callback)
        self.raw_demos = poor_demonstrations
        self.labeled_demos = []
        self.task = task
        self.output_directory = output_directory
        self.subject = subject

    def run(self):
        rospy.loginfo("Running the ARPO LfD Experiment Controller...")
        rospy.loginfo(
            "This depends on two keyboard input nodes from cairo_lfd: modeling_keyboard_commands.py & recording_keyboard_commands.py")
        while self.command != "quit" and not rospy.is_shutdown():
            if self.command == "resample":
                rospy.loginfo("Resampling keyframe models...")
                self._clear_command()
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                    "number_of_samples", 25), automated_culling=False)
            if self.command == "get_representation":
                rospy.loginfo(
                    "Publishing keyframe model representation to cairo_lfd/lfd_representation keyframe models...")
                self._clear_command()
                data = self.lfd_model.generate_representation()
                self.representation_publisher.publish(json.dumps(data))
            if self.command == "execute":
                rospy.loginfo("Executing learned model...")
                self._clear_command()
                self._move_to_start_configuration()
                self.lfd_model.perform_skill()
            if self.command == "start":
                rospy.loginfo("Moving to start configuration...")
                self._clear_command()
                self._move_to_start_configuration()
            if self.command == "calibrate":
                rospy.loginfo("Moving to calibration pose...")
                self._clear_command()
                self._move_to_calibration_pose()
            if self.command == "record":
                rospy.loginfo("Entering recording mode...")
                self.raw_demos.extend(self.recorder.record())
                self.labeled_demos = self.labeler.label(self.raw_demos)
                self._clear_command()
                rospy.loginfo("Returned from recording mode...")
                rospy.loginfo("There are now {} aligned and labeled demonstrations available for learning.".format(
                    len(self.labeled_demos)))
            if self.command == "train":
                rospy.loginfo(
                    "Retraining entire model using current demonstrations...")
                self.labeled_demos = self.labeler.label(self.raw_demos)
                self.lfd_model.build_keyframe_graph(
                    self.labeled_demos, self.lfd_model.settings.get("bandwidth", .025))
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                    "number_of_samples", 25), automated_culling=True)
                rospy.loginfo(
                    "Training complete. New keyframe model available for representation and execution.")
                self._clear_command()
            if self.command == "save":
                rospy.loginfo("Saving subject data...")
                self.save_trial_data()
                self._clear_command()
            if self.command == "serialize":
                rospy.loginfo("Serializing model...")
                unique_filename = str(uuid.uuid4())
                dirname = './' + self.output_directory + '/' + self.task + '/' + self.subject
                if not os.path.exists(dirname):
                    os.makedirs(dirname)
                path = dirname + '/serialization_' + unique_filename + '.json'
                self.lfd_model.serialize_out(path)
                self._clear_command()

    def save_trial_data(self):
        dirname_raw = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/raw'
        if not os.path.exists(dirname_raw):
            os.makedirs(dirname_raw)
        rospy.loginfo("Saving raw demos to '{}/'".format(dirname_raw))
        dirname_labeled = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/labeled'
        if not os.path.exists(dirname_labeled):
            os.makedirs(dirname_labeled)
        rospy.loginfo("Saving labeled demos to '{}/'".format(dirname_labeled))
        for idx, demo in enumerate(self.raw_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            export_to_json(
                dirname_raw + "/raw_demo_{}.json".format(unique_filename), raw_data)
        for idx, demo in enumerate(self.labeled_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.labeled_observations]
            export_to_json(
                dirname_labeled + "/labeled_demo_{}.json".format(unique_filename), raw_data)

    def _command_callback(self, msg):
        self.command = msg.data

    # def _update_keyframe_callback(self, msg):
    #     unity_json_data = json.loads(msg.data)
    #     parsed_data = {}
    #     for node, data in unity_json_data.items():
    #         parsed_data[int(node)] = {"applied_constraints": [int(
    #             value) for value in data["applied_constraints"] if "applied_constraints" in data.keys()]}
    #     self.lfd_model.update_applied_constraints(parsed_data)
    #     # Sample and refit existing keyframe models.
    #     self.lfd_model.sample_keyframes(
    #         self.lfd_model.settings.get("number_of_samples", 25))

    # def _update_constraints_callback(self, msg):
    #     unity_json_data = json.loads(msg.data)
    #     constraint_configs = remap_constraints_for_lfd(unity_json_data)
    #     self.lfd_model.update_constraints(constraint_configs)
    
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
        
    def _move_to_calibration_pose(self):
        if self.calibration_pose is not None:
            pose = convert_data_to_pose(self.calibration_pose['position'], self.calibration_pose['orientation'])
            moveit_interface = SawyerMoveitInterface()
            moveit_interface.set_velocity_scaling(.35)
            moveit_interface.set_acceleration_scaling(.25)
            moveit_interface.set_pose_target(pose)
            moveit_interface.execute(moveit_interface.plan())
        else:
            rospy.logwarn("No start configuration provided in your config.json file.")
        self._clear_command()
        

    def _clear_command(self):
        self.command = ""



class ARStudyController():
    """

    Attributes
    ----------

    """

    def __init__(self, calibration_settings, lfd_model, recorder, labeler, poor_demonstrations, output_directory, task, subject):
        """
        Parameters
        ----------

        """
        self.command = ""
        self.calibration_pose = calibration_settings.get("calibration_pose", None)
        self.start_configuration = calibration_settings.get("start_configuration", None)
        self.lfd_model = lfd_model
        self.recorder = recorder
        self.labeler = labeler
        self.keyframe_update_subscriber = rospy.Subscriber(
            '/cairo_lfd/keyframe_update', String, self._update_keyframe_callback)
        self.constraint_update_subscriber = rospy.Subscriber(
            '/cairo_lfd/constraint_update', String, self._update_constraints_callback)
        self.command_subscriber = rospy.Subscriber(
            '/cairo_lfd/model_commands', String, self._command_callback)
        self.representation_publisher = rospy.Publisher(
            '/cairo_lfd/lfd_representation', String, queue_size=10)
        self.raw_demos = poor_demonstrations
        self.labeled_demos = []
        self.task = task
        self.output_directory = output_directory
        self.subject = subject

    def run(self):
        rospy.loginfo("Running the AR 4 LfD Experiment Controller...")
        rospy.loginfo(
            "This depends on two keyboard input nodes from cairo_lfd: modeling_keyboard_commands.py & recording_keyboard_commands.py")
        while self.command != "quit" and not rospy.is_shutdown():
            if self.command == "resample":
                rospy.loginfo("Resampling keyframe models...")
                self._clear_command()
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                    "number_of_samples", 25), automated_culling=False)
            if self.command == "get_representation":
                rospy.loginfo(
                    "Publishing keyframe model representation to cairo_lfd/lfd_representation keyframe models...")
                self._clear_command()
                data = self.lfd_model.generate_representation()
                self.representation_publisher.publish(json.dumps(data))
            if self.command == "execute":
                rospy.loginfo("Executing learned model...")
                self._clear_command()
                self._move_to_start_configuration()
                self.lfd_model.perform_skill()
            if self.command == "start":
                rospy.loginfo("Moving to start configuration...")
                self._clear_command()
                self._move_to_start_configuration()
            if self.command == "calibrate":
                rospy.loginfo("Moving to calibration pose...")
                self._clear_command()
                self._move_to_calibration_pose()
            if self.command == "record":
                rospy.loginfo("Entering recording mode...")
                self.raw_demos.extend(self.recorder.record())
                self.labeled_demos = self.labeler.label(self.raw_demos)
                self._clear_command()
                rospy.loginfo("Returned from recording mode...")
                rospy.loginfo("There are now {} aligned and labeled demonstrations available for learning.".format(
                    len(self.labeled_demos)))
            if self.command == "train":
                rospy.loginfo(
                    "Retraining entire model using current demonstrations...")
                self.labeled_demos = self.labeler.label(self.raw_demos)
                self.lfd_model.build_keyframe_graph(
                    self.labeled_demos, self.lfd_model.settings.get("bandwidth", .025))
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                    "number_of_samples", 25), automated_culling=True)
                rospy.loginfo(
                    "Training complete. New keyframe model available for representation and execution.")
                self._clear_command()
            if self.command == "save":
                rospy.loginfo("Saving subject data...")
                self.save_trial_data()
                self._clear_command()
            if self.command == "serialize":
                rospy.loginfo("Serializing model...")
                unique_filename = str(uuid.uuid4())
                dirname = './' + self.output_directory + '/' + self.task + '/' + self.subject
                if not os.path.exists(dirname):
                    os.makedirs(dirname)
                path = dirname + '/serialization_' + unique_filename + '.json'
                self.lfd_model.serialize_out(path)
                self._clear_command()

    def save_trial_data(self):
        if self.task is None or self.subject is None:
            dirname_raw = './' + self.output_directory
        else:
            dirname_raw = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/raw'
        if not os.path.exists(dirname_raw):
            os.makedirs(dirname_raw)
        rospy.loginfo("Saving raw demos to '{}/'".format(dirname_raw))
        dirname_labeled = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/labeled'
        if not os.path.exists(dirname_labeled):
            os.makedirs(dirname_labeled)
        rospy.loginfo("Saving labeled demos to '{}/'".format(dirname_labeled))
        for idx, demo in enumerate(self.raw_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            export_to_json(
                dirname_raw + "/raw_demo_{}.json".format(unique_filename), raw_data)
        for idx, demo in enumerate(self.labeled_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.labeled_observations]
            export_to_json(
                dirname_labeled + "/labeled_demo_{}.json".format(unique_filename), raw_data)

    def _command_callback(self, msg):
        self.command = msg.data

    def _update_keyframe_callback(self, msg):
        unity_json_data = json.loads(msg.data)
        parsed_data = {}
        for node, data in unity_json_data.items():
            parsed_data[int(node)] = {"applied_constraints": [int(
                value) for value in data["applied_constraints"] if "applied_constraints" in data.keys()]}
        self.lfd_model.update_applied_constraints(parsed_data)
        # Sample and refit existing keyframe models.
        self.lfd_model.sample_keyframes(
            self.lfd_model.settings.get("number_of_samples", 25))

    def _update_constraints_callback(self, msg):
        unity_json_data = json.loads(msg.data)
        constraint_configs = remap_constraints_for_lfd(unity_json_data)
        self.lfd_model.update_constraints(constraint_configs)
    
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
        
    def _move_to_calibration_pose(self):
        if self.calibration_pose is not None:
            pose = convert_data_to_pose(self.calibration_pose['position'], self.calibration_pose['orientation'])
            moveit_interface = SawyerMoveitInterface()
            moveit_interface.set_velocity_scaling(.35)
            moveit_interface.set_acceleration_scaling(.25)
            moveit_interface.set_pose_target(pose)
            moveit_interface.execute(moveit_interface.plan())
        else:
            rospy.logwarn("No start configuration provided in your config.json file.")
        self._clear_command()
        

    def _clear_command(self):
        self.command = ""


class RecordingOnlyController():
    """

    Attributes
    ----------

    """

    def __init__(self, lfd_model, recorder, output_directory):
        """
        Parameters
        ----------

        """
        self.lfd_model = lfd_model
        self.recorder = recorder
        self.raw_demos = []
        self.labeled_demos = []
        self.output_directory = output_directory

    def run(self):
        rospy.loginfo("Entering recording mode...")
        self.raw_demos.extend(self.recorder.record())
        rospy.loginfo("Returned from recording mode...")
        rospy.loginfo("There are now {} raw demonstrations available for learning.".format(
            len(self.raw_demos)))
        self.save_recording_data()

    def save_recording_data(self):
        dirname_raw = './' + self.output_directory + '/raw'
        if not os.path.exists(dirname_raw):
            os.makedirs(dirname_raw)
        rospy.loginfo("Saving raw demos to '{}/'".format(dirname_raw))
        for idx, demo in enumerate(self.raw_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            export_to_json(
                dirname_raw + "/raw_demo_{}.json".format(unique_filename), raw_data)


class ACCLfDController():
    """

    Attributes
    ----------

    """

    def __init__(self, acclfd_model, recorder, labeler, initial_demonstrations, output_directory, task, subject):
        """
        Parameters
        ----------

        """
        self.command = ""
        self.lfd_model = acclfd_model
        self.recorder = recorder
        self.labeler = labeler
        self.command_subscriber = rospy.Subscriber(
            '/cairo_lfd/model_commands', String, self._command_callback)
        self.representation_publisher = rospy.Publisher(
            '/cairo_lfd/lfd_representation', String, queue_size=10)
        self.raw_demos = initial_demonstrations
        self.labeled_demos = []
        self.task = task
        self.output_directory = output_directory
        self.subject = subject

    def run(self):
        rospy.loginfo("Running the ACC LfD Experiment Controller...")
        rospy.loginfo(
            "This depends on two keyboard input nodes from cairo_lfd: modeling_keyboard_commands.py & recording_keyboard_commands.py")
        while self.command != "quit" and not rospy.is_shutdown():
            if self.command == "resample":
                rospy.loginfo("Resampling keyframe models...")
                self._clear_command()
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                    "number_of_samples", .025), automated_culling=False)
            if self.command == "get_representation":
                rospy.loginfo(
                    "Publishing keyframe model representation to cairo_lfd/lfd_representation keyframe models...")
                self._clear_command()
                data = self.lfd_model.generate_representation()
                self.representation_publisher.publish(json.dumps(data))
            if self.command == "execute":
                rospy.loginfo("Executing learned model...")
                self._clear_command()
                self.lfd_model.perform_skill()
            if self.command == "record":
                rospy.loginfo("Entering recording mode...")
                self.raw_demos.extend(self.recorder.record())
                self._clear_command()
                rospy.loginfo("Returned from recording mode...")
                rospy.loginfo("There are now {} raw demonstrations available for training.".format(
                    len(self.raw_demos)))
            if self.command == "train":
                if len(self.raw_demos) > 1:
                    rospy.loginfo(
                        "Retraining entire model using current raw demonstrations...")
                    self.labeled_demos = self.labeler.label(self.raw_demos)
                    self.lfd_model.build_keyframe_graph(
                        self.labeled_demos, self.lfd_model.settings.get("bandwidth", .025))
                    self.lfd_model.generate_autoconstraints(
                        self.labeled_demos)
                    self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                        "number_of_samples", .025), automated_culling=True)
                    rospy.loginfo(
                        "Training complete. New keyframe model available for representation and execution.")
                else:
                    rospy.loginfo("No demonstrations available for training.")
                self._clear_command()
            if self.command == "save":
                rospy.loginfo("Saving subject data...")
                self.save_trial_data()
                self._clear_command()
            if self.command == "serialize":
                rospy.loginfo("Serializing model...")
                unique_filename = str(uuid.uuid4())
                dirname = './' + self.output_directory + '/' + self.task + '/' + self.subject
                if not os.path.exists(dirname):
                    os.makedirs(dirname)
                path = dirname + '/serialization_' + unique_filename + '.json'
                self.lfd_model.serialize_out(path)
                self._clear_command()

    def save_trial_data(self):
        dirname_raw = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/raw'
        if not os.path.exists(dirname_raw):
            os.makedirs(dirname_raw)
        rospy.loginfo("Saving raw demos to '{}/'".format(dirname_raw))
        dirname_labeled = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/labeled'
        if not os.path.exists(dirname_labeled):
            os.makedirs(dirname_labeled)
        rospy.loginfo("Saving labeled demos to '{}/'".format(dirname_labeled))
        for idx, demo in enumerate(self.raw_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            export_to_json(
                dirname_raw + "/raw_demo_{}.json".format(unique_filename), raw_data)
        for idx, demo in enumerate(self.labeled_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.labeled_observations]
            export_to_json(
                dirname_labeled + "/labeled_demo_{}.json".format(unique_filename), raw_data)

    def _command_callback(self, msg):
        self.command = msg.data

    def _clear_command(self):
        self.command = ""


class CCLfDController():
    """

    Attributes
    ----------

    """

    def __init__(self, calibration_settings, cclfd_model, recorder, labeler, raw_demos, labeled_demos, output_directory, task, subject):
        """
        Parameters
        ----------

        """
        self.command = ""
        self.lfd_model = cclfd_model
        self.start_configuration = calibration_settings.get("start_configuration", None)
        self.recorder = recorder
        self.labeler = labeler
        self.update_subscriber = rospy.Subscriber(
            '/cairo_lfd/model_update', String, self._update_callback)
        self.command_subscriber = rospy.Subscriber(
            '/cairo_lfd/model_commands', String, self._command_callback)
        self.representation_publisher = rospy.Publisher(
            '/cairo_lfd/lfd_representation', String, queue_size=10)
        self.raw_demos = raw_demos
        self.labeled_demos = labeled_demos
        if self.labeled_demos != []:
            self.ready_to_serialize = True
        else:
            self.ready_to_serialize = False
        self.task = task
        self.output_directory = output_directory
        self.subject = subject

    def run(self):
        rospy.loginfo("Running the CC LfD Experiment Controller...")
        rospy.loginfo(
            "This depends on two keyboard input nodes from cairo_lfd: modeling_keyboard_commands.py & recording_keyboard_commands.py")
        while self.command != "quit" and not rospy.is_shutdown():
            if self.command == "resample":
                rospy.loginfo("Resampling keyframe models...")
                self._clear_command()
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                    "number_of_samples", .025), automated_culling=False)
            if self.command == "get_representation":
                rospy.loginfo(
                    "Publishing keyframe model representation to cairo_lfd/lfd_representation keyframe models...")
                self._clear_command()
                data = self.lfd_model.generate_representation()
                self.representation_publisher.publish(json.dumps(data))
            if self.command == "execute":
                rospy.loginfo("Executing learned model...")
                self._clear_command()
                self.lfd_model.perform_skill()
            if self.command == "start":
                rospy.loginfo("Moving to start configuration...")
                self._clear_command()
                self._move_to_start_configuration()
            if self.command == "record":
                rospy.loginfo("Entering recording mode...")
                self.raw_demos.extend(self.recorder.record())
                self._clear_command()
                rospy.loginfo("Returned from recording mode...")
                rospy.loginfo("There are now {} raw demonstrations available for training.".format(
                    len(self.raw_demos)))
            if self.command == "train":
                if len(self.raw_demos) > 1:
                    rospy.loginfo(
                        "Retraining entire model using current raw demonstrations...")
                    self.labeled_demos = self.labeler.label(self.raw_demos)
                    self.lfd_model.build_keyframe_graph(
                        self.labeled_demos, self.lfd_model.settings.get("bandwidth", .025))
                    self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                        "number_of_samples", .025), automated_culling=True)
                    rospy.loginfo(
                        "Training complete. New keyframe model available for representation and execution.")
                    self.ready_to_serialize = True
                else:
                    rospy.loginfo("No demonstrations available for training.")
                self._clear_command()
            if self.command == "save":
                rospy.loginfo("Saving subject data...")
                if self.raw_demos != []:
                    self.save_trial_data()
                else:
                    rospy.loginfo("No data to save!")
                self._clear_command()
            if self.command == "serialize":
                if self.ready_to_serialize is False:
                    rospy.loginfo("No model available to seralize...train the model first.") 
                else:
                    rospy.loginfo("Serializing model...")
                    unique_filename = str(uuid.uuid4())
                    dirname = './' + self.output_directory + '/' + self.task + '/' + self.subject
                    if not os.path.exists(dirname):
                        os.makedirs(dirname)
                    path = dirname + '/serialization_' + unique_filename + '.json'
                    self.lfd_model.serialize_out(path)
                self._clear_command()

    def save_trial_data(self):
        dirname_raw = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/raw'
        if not os.path.exists(dirname_raw):
            os.makedirs(dirname_raw)
        rospy.loginfo("Saving raw demos to '{}/'".format(dirname_raw))
        dirname_labeled = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/labeled'
        if not os.path.exists(dirname_labeled):
            os.makedirs(dirname_labeled)
        if self.labeled_demos == []:
            rospy.loginfo(
                "Labeling raw demos in order to save labeled demosntrations...")
            self.labeled_demos = self.labeler.label(self.raw_demos)
        rospy.loginfo("Saving labeled demos to '{}/'".format(dirname_labeled))
        for idx, demo in enumerate(self.raw_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            export_to_json(
                dirname_raw + "/raw_demo_{}.json".format(unique_filename), raw_data)
        for idx, demo in enumerate(self.labeled_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.labeled_observations]
            export_to_json(
                dirname_labeled + "/labeled_demo_{}.json".format(unique_filename), raw_data)

    def _command_callback(self, msg):
        self.command = msg.data

    def _update_callback(self, msg):
        self.lfd_model.model_update(json.loads(msg.data))
        # Sample and refit existing keyframe models.
        self.lfd_model.sample_keyframes(
            self.lfd_model.settings.get("number_of_samples", .025))

    def _clear_command(self):
        self.command = ""
    
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


class LfDController():
    """

    Attributes
    ----------

    """

    def __init__(self, lfd_model, recorder, labeler, initial_demonstrations, output_directory, task, subject):
        """
        Parameters
        ----------

        """
        self.command = ""
        self.lfd_model = lfd_model
        self.recorder = recorder
        self.labeler = labeler
        self.command_subscriber = rospy.Subscriber(
            '/cairo_lfd/model_commands', String, self._command_callback)
        self.representation_publisher = rospy.Publisher(
            '/cairo_lfd/lfd_representation', String, queue_size=10)
        self.raw_demos = initial_demonstrations
        self.labeled_demos = []
        self.task = task
        self.output_directory = output_directory
        self.subject = subject

    def run(self):
        rospy.loginfo("Running the ACC LfD Experiment Controller...")
        rospy.loginfo(
            "This depends on two keyboard input nodes from cairo_lfd: modeling_keyboard_commands.py & recording_keyboard_commands.py")
        while self.command != "quit" and not rospy.is_shutdown():
            if self.command == "resample":
                rospy.loginfo("Resampling keyframe models...")
                self._clear_command()
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                    "number_of_samples", .025), automated_culling=False)
            if self.command == "get_representation":
                rospy.loginfo(
                    "Publishing keyframe model representation to cairo_lfd/lfd_representation keyframe models...")
                self._clear_command()
                data = self.lfd_model.generate_representation()
                self.representation_publisher.publish(json.dumps(data))
            if self.command == "execute":
                rospy.loginfo("Executing learned model...")
                self._clear_command()
                self.lfd_model.perform_skill()
            if self.command == "record":
                rospy.loginfo("Entering recording mode...")
                self.raw_demos.extend(self.recorder.record())
                self._clear_command()
                rospy.loginfo("Returned from recording mode...")
                rospy.loginfo("There are now {} raw demonstrations available for training.".format(
                    len(self.raw_demos)))
            if self.command == "train":
                if len(self.raw_demos) > 1:
                    rospy.loginfo(
                        "Retraining entire model using current raw demonstrations...")
                    self.labeled_demos = self.labeler.label(self.raw_demos)
                    self.lfd_model.build_keyframe_graph(
                        self.labeled_demos, self.lfd_model.settings.get("bandwidth", .025))
                    self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                        "number_of_samples", .025), automate_threshold=True)
                    rospy.loginfo(
                        "Training complete. New keyframe model available for representation and execution.")
                else:
                    rospy.loginfo("No demonstrations available for training.")
                self._clear_command()
            if self.command == "save":
                rospy.loginfo("Saving subject data...")
                self.save_trial_data()
                self._clear_command()
            if self.command == "serialize":
                rospy.loginfo("Serializing model...")
                unique_filename = str(uuid.uuid4())
                dirname = './' + self.output_directory + '/' + self.task + '/' + self.subject
                if not os.path.exists(dirname):
                    os.makedirs(dirname)
                path = dirname + '/serialization_' + unique_filename + '.json'
                self.lfd_model.serialize_out(path)
                self._clear_command()

    def save_trial_data(self):
        dirname_raw = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/raw'
        if not os.path.exists(dirname_raw):
            os.makedirs(dirname_raw)
        rospy.loginfo("Saving raw demos to '{}/'".format(dirname_raw))
        dirname_labeled = './' + self.output_directory + \
            '/' + self.task + '/' + self.subject + '/labeled'
        if not os.path.exists(dirname_labeled):
            os.makedirs(dirname_labeled)
        rospy.loginfo("Saving labeled demos to '{}/'".format(dirname_labeled))
        for idx, demo in enumerate(self.raw_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            export_to_json(
                dirname_raw + "/raw_demo_{}.json".format(unique_filename), raw_data)
        for idx, demo in enumerate(self.labeled_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.labeled_observations]
            export_to_json(
                dirname_labeled + "/labeled_demo_{}.json".format(unique_filename), raw_data)

    def _command_callback(self, msg):
        self.command = msg.data

    def _clear_command(self):
        self.command = ""


class FeedbackLfDStudyController():
    """

    Attributes
    ----------

    """

    def __init__(self, lfd_model, recorder, labeler, initial_demonstrations, output_directory):
        """
        Parameters
        ----------

        """
        self.command = ""
        self.lfd_model = lfd_model
        self.recorder = recorder
        self.labeler = labeler
        self.update_subscriber = rospy.Subscriber(
            '/cairo_lfd/model_update', String, self._update_callback)
        self.command_subscriber = rospy.Subscriber(
            '/cairo_lfd/model_commands', String, self._command_callback)
        self.representation_publisher = rospy.Publisher(
            '/cairo_lfd/lfd_representation', String, queue_size=10)
        self.raw_demos = initial_demonstrations
        self.labeled_demos = []
        self.output_directory = output_directory

    def run(self):
        rospy.loginfo("Running the Feeback LfD Experiment Controller...")
        rospy.loginfo(
            "This depends on two keyboard input nodes from cairo_lfd: modeling_keyboard_commands.py & recording_keyboard_commands.py")
        while self.command != "quit" and not rospy.is_shutdown():
            if self.command == "resample":
                rospy.loginfo("Resampling keyframe models...")
                self._clear_command()
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                    "number_of_samples", .025), automated_culling=False)
            if self.command == "get_representation":
                rospy.loginfo(
                    "Publishing keyframe model representation to cairo_lfd/lfd_representation keyframe models...")
                self._clear_command()
                data = self.lfd_model.generate_representation()
                self.representation_publisher.publish(json.dumps(data))
            if self.command == "execute":
                rospy.loginfo("Executing learned model...")
                self._clear_command()
                self.lfd_model.perform_skill()
            if self.command == "record":
                rospy.loginfo("Entering recording mode...")
                self.raw_demos.extend(self.recorder.record())
                self.labeled_demos = self.labeler.label(self.raw_demos)
                self._clear_command()
                rospy.loginfo("Returned from recording mode...")
                rospy.loginfo("There are now {} aligned and labeled demonstrations available for learning.".format(
                    len(self.labeled_demos)))
            if self.command == "train":
                rospy.loginfo(
                    "Retraining entire model using current demonstrations...")
                self.labeled_demos = self.labeler.label(self.raw_demos)
                self.lfd_model.build_keyframe_graph(
                    self.labeled_demos, self.lfd_model.settings.get("bandwidth", .025))
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get(
                    "number_of_samples", .025), automated_culling=True)
                rospy.loginfo(
                    "Training complete. New keyframe model available for representation and execution.")
                self._clear_command()
            if self.command == "save":
                rospy.loginfo("Saving subject data...")
                self.save_trial_data()
                self._clear_command()
            if self.command == "serialize":
                rospy.loginfo("Serializing model...")
                unique_filename = str(uuid.uuid4())
                dirname = './' + self.output_directory + '/' + self.task + '/' + self.subject
                if not os.path.exists(dirname):
                    os.makedirs(dirname)
                path = dirname + '/serialization_' + unique_filename + '.json'
                self.lfd_model.serialize_out(path)
                self._clear_command()

    def save_trial_data(self):
        dirname_raw = './' + self.output_directory + '/raw'
        if not os.path.exists(dirname_raw):
            os.makedirs(dirname_raw)
        rospy.loginfo("Saving raw demos to '{}/'".format(dirname_raw))
        dirname_labeled = './' + self.output_directory + '/labeled'
        if not os.path.exists(dirname_labeled):
            os.makedirs(dirname_labeled)
        rospy.loginfo("Saving labeled demos to '{}/'".format(dirname_labeled))
        for idx, demo in enumerate(self.raw_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            export_to_json(
                dirname_raw + "/raw_demo_{}.json".format(unique_filename), raw_data)
        for idx, demo in enumerate(self.labeled_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.labeled_observations]
            export_to_json(
                dirname_labeled + "/labeled_demo_{}.json".format(unique_filename), raw_data)

    def _command_callback(self, msg):
        self.command = msg.data

    def _update_callback(self, msg):
        unity_json_data = json.loads(msg.data)
        parsed_data = {}
        for node, data in unity_json_data.items():
            parsed_data[int(node)] = {"applied_constraints": [int(
                value) for value in data["applied_constraints"] if "applied_constraints" in data.keys()]}
        # TODO need to update lfd_model to accept an append command
        self.lfd_model.model_update(parsed_data)
        # Sample and refit existing keyframe models.
        self.lfd_model.sample_keyframes(
            self.lfd_model.settings.get("number_of_samples", .025))

    def _clear_command(self):
        self.command = ""
