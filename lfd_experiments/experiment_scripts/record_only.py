#!/usr/bin/env python3
import os
import argparse
from functools import partial

import rospy

import intera_interface
from intera_interface import CHECK_VERSION

from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.core.record import SawyerDemonstrationRecorder
from cairo_lfd.data.io import load_lfd_configuration
from cairo_lfd.data.processing import DataProcessingPipeline, RelativeKinematicsProcessor, RelativePositionProcessor, InContactProcessor, SphereOfInfluenceProcessor, WithinPerimeterProcessor
from cairo_lfd.core.lfd import CC_LFD
from cairo_lfd.controllers.study_controllers import RecordingOnlyController


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-c', '--config', dest='config', required=True,
        help='the file path of the demonstration '
    )

    required.add_argument(
        '-o', '--output_directory', dest='output_directory', required=True,
        help='the directory to save the given subjects .json files data'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    ############################
    #  ROS Node Initialization #
    ############################

    print("Initializing node... ")
    rospy.init_node("recording_only")
    print("Getting robot state... ")
    robot_state = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    robot_state.enable()

    ########################
    # Import Configuration #
    ########################

    config_filepath = args.config
    configs = load_lfd_configuration(config_filepath)

    #################################
    # Configure the LFD class model #
    #################################

    model_settings = configs["settings"]["modeling_settings"]
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.35)
    moveit_interface.set_acceleration_scaling(.25)
    moveit_interface.set_planner(str(model_settings["planner"]))
    cclfd = CC_LFD(configs, model_settings, moveit_interface)
    cclfd.build_environment()

    #####################################
    # Raw Demonstration Data Processors #
    #####################################

    # Build processors and process demonstrations to generate derivative data e.g. relative position.
    rk_processor = RelativeKinematicsProcessor(cclfd.environment.get_item_ids(), cclfd.environment.get_robot_id())
    ic_processor = InContactProcessor(cclfd.environment.get_item_ids(), cclfd.environment.get_robot_id(), .06, .5)
    soi_processor = SphereOfInfluenceProcessor(cclfd.environment.get_item_ids(), cclfd.environment.get_robot_id())
    rp_processor = RelativePositionProcessor(cclfd.environment.get_item_ids(), cclfd.environment.get_robot_id())
    wp_processor = WithinPerimeterProcessor(cclfd.environment.get_item_ids(), cclfd.environment.get_robot_id())
    processor_pipeline = DataProcessingPipeline([rk_processor, ic_processor, soi_processor, rp_processor, wp_processor])

    ###############################################
    # Configure the Sawyer Demonstration Recorder #
    ###############################################

    rec_settings = configs["settings"]["recording_settings"]
    recorder = SawyerDemonstrationRecorder(rec_settings, cclfd.environment, processor_pipeline, publish_constraint_validity=True)
    rospy.on_shutdown(recorder.stop)

    recording_controller = RecordingOnlyController(cclfd, recorder, args.output_directory)
    recording_controller.run()


if __name__ == '__main__':
    main()