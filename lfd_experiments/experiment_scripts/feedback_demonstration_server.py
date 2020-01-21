#!/usr/bin/python

import os
import os
import argparse
from functools import partial
import random
import colored_traceback
import pudb
import numpy as np
import rospy

from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.core.lfd import CC_LFD
from cairo_lfd.core.environment import Observation, Demonstration
from cairo_lfd.data.io import DataImporter, import_configuration
from lfd_experiments.srv import PerformDemonstration

colored_traceback.add_hook()


class PerformDemonstrationServer():

    def __init__(self):
        rospy.init_node('feedback_demonstration_server')

    def main(self):
        rospy.Service("feedback_demonstration", PerformDemonstration, self.handle)
        rospy.loginfo("CC-LfD: Service up!")
        rospy.spin()

    def handle(self, constraint_type):
        # Get filepath to demo data
        main_filepath = rospy.get_param("MAIN_FILEPATH")
        config_filepath = main_filepath + "/config.json"
        pos_directory = main_filepath + "/positive/labeled"
        neg_directory = main_filepath + "/negative/labeled"

        # Set data directory by constraint value
        if constraint_type.constraint == 0:
            directory = neg_directory
        else:
            directory = pos_directory

        # Setup LfD parameters
        bandwidth = 0.025
        number_of_samples = 5

        rospy.loginfo("CC-LfD: %s" % str(constraint_type.constraint))

        # Import the data
        importer = DataImporter()
        configs = import_configuration(config_filepath)

        labeled_demonstrations = importer.load_json_files(directory + "/*.json")

        # Convert imported data into Demonstrations and Observations
        demonstrations = []
        for datum in labeled_demonstrations["data"]:
            observations = []
            for entry in datum:
                observations.append(Observation(entry))
            demonstrations.append(Demonstration(observations))

        if len(demonstrations) == 0:
            rospy.logwarn("No demonstration data to model!!")
            return False

        """ Create the moveit_interface """
        moveit_interface = SawyerMoveitInterface()
        moveit_interface.set_velocity_scaling(.35)
        moveit_interface.set_acceleration_scaling(.25)

        cclfd = CC_LFD(configs, moveit_interface)
        cclfd.build_environment()
        cclfd.build_keyframe_graph(demonstrations, bandwidth)
        cclfd.sample_keyframes(number_of_samples)
        cclfd.perform_skill()

        return True


if __name__ == '__main__':
    try:
        obj = PerformDemonstrationServer()
        obj.main()
    except rospy.ROSInterruptException:
        pass
