import os
import json
import uuid

import rospy
from std_msgs.msg import String

from cairo_lfd.data.io import DataExporter


class ARStudyController():
    """

    Attributes
    ----------

    """
    def __init__(self, lfd_model, recorder, labeler, poor_demonstrations, output_directory, task, subject):
        """
        Parameters
        ----------

        """
        self.command = ""
        self.lfd_model = lfd_model
        self.recorder = recorder
        self.labeler = labeler
        self.update_subscriber = rospy.Subscriber('/cairo_lfd/model_update', String, self._update_callback)
        self.command_subscriber = rospy.Subscriber('/cairo_lfd/model_commands', String, self._command_callback)
        self.representation_publisher = rospy.Publisher('/cairo_lfd/lfd_representation', String, queue_size=10)
        self.raw_demos = poor_demonstrations
        self.labeled_demos = []
        self.task = task
        self.output_directory = output_directory
        self.subject = subject

    def run(self):
        rospy.loginfo("Running the AR 4 LfD Experiment Controller...")
        rospy.loginfo("This depends on two keyboard input nodes from cairo_lfd: modeling_keyboard_commands.py & recording_keyboard_commands.py")
        while self.command != "quit" and not rospy.is_shutdown():
            if self.command == "resample":
                rospy.loginfo("Resampling keyframe models...")
                self._clear_command()
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get("number_of_samples", .025), automate_threshold=False)
            if self.command == "get_representation":
                rospy.loginfo("Publishing keyframe model representation to cairo_lfd/lfd_representation keyframe models...")
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
                rospy.loginfo("There are now {} aligned and labeled demonstrations available for learning.".format(len(self.labeled_demos)))
            if self.command == "train":
                rospy.loginfo("Retraining entire model using current demonstrations...")
                self.lfd_model.build_keyframe_graph(self.labeled_demos, self.lfd_model.settings.get("bandwidth", .025))
                self.lfd_model.sample_keyframes(self.lfd_model.settings.get("number_of_samples", .025), automate_threshold=True)
                rospy.loginfo("Training complete. New keyframe model available for representation and execution.")
                self._clear_command()
            if self.command == "save":
                rospy.loginfo("Saving subject data...")
                self.save_trial_data()
                self._clear_command()

    def save_trial_data(self):
        exp = DataExporter()
        dirname_raw = './' + self.output_directory + '/' + self.task + '/' + self.subject + '/raw'
        if not os.path.exists(dirname_raw):
            os.makedirs(dirname_raw)
        rospy.loginfo("Saving raw demos to '{}/'".format(dirname_raw))
        dirname_labeled = './' + self.output_directory + '/' + self.task + '/' + self.subject + '/labeled'
        if not os.path.exists(dirname_labeled):
            os.makedirs(dirname_labeled)
        rospy.loginfo("Saving labeled demos to '{}/'".format(dirname_labeled))
        for idx, demo in enumerate(self.raw_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            exp.export_to_json(dirname_raw + "/raw_demo_{}.json".format(unique_filename), raw_data)
        for idx, demo in enumerate(self.labeled_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            exp.export_to_json(dirname_labeled + "/labeled_demo_{}.json".format(unique_filename), raw_data)

    def _command_callback(self, msg):
        self.command = msg.data

    def _update_callback(self, msg):
        self.lfd_model.model_update(json.loads(msg.data))
        # Sample and refit existing keyframe models.
        self.lfd_model.sample_keyframes(self.lfd_model.settings.get("number_of_samples", .025))

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
        rospy.loginfo("There are now {} raw demonstrations available for learning.".format(len(self.raw_demos)))
        self.save_recording_data()

    def save_recording_data(self):
        exp = DataExporter()
        dirname_raw = './' + self.output_directory + '/raw'
        if not os.path.exists(dirname_raw):
            os.makedirs(dirname_raw)
        rospy.loginfo("Saving raw demos to '{}/'".format(dirname_raw))
        for idx, demo in enumerate(self.raw_demos):
            unique_filename = str(uuid.uuid4())
            raw_data = [obs.data for obs in demo.observations]
            exp.export_to_json(dirname_raw + "/raw_demo_{}.json".format(unique_filename), raw_data)
