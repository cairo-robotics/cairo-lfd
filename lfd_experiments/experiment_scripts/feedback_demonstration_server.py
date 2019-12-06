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
        rospy.init_node('perform_demonstration_server')

    def run(self):
        service = rospy.Service("perform_demonstration", PerformDemonstration, self.handle)
        rospy.loginfo("CC-LfD: Service up!")
        rospy.spin()

    def handle(self, constraint_type):
        curr_file = os.path.dirname(os.path.abspath(__file__))
        config_filepath = curr_file + "/../experiment_data/cc_lfd/feedback_demo/config.json"
        pos_directory = curr_file + "/../experiment_data/cc_lfd/feedback_demo/positive/labeled"
        neg_directory = curr_file + "/../experiment_data/cc_lfd/feedback_demo/negative/labeled"
        bandwidth = 0.025
        number_of_samples = 50

        rospy.logwarn(str(constraint_type.constraint))

        if constraint_type.constraint == 0:
            directory = neg_directory
        else:
            directory = pos_directory

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
            return 0

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


def main():
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__)
    # required = parser.add_argument_group('required arguments')

    # args = parser.parse_args(rospy.myargv()[1:])

    try:
        obj = PerformDemonstrationServer()
        obj.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
