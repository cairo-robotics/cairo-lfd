#!/usr/bin/env python
import os
import argparse
from functools import partial

import rospy

import intera_interface
from intera_interface import CHECK_VERSION

from robot_interface.moveit_interface import SawyerMoveitInterface
from cairo_lfd.core.record import SawyerDemonstrationRecorder, SawyerDemonstrationLabeler
from cairo_lfd.core.environment import Environment, Observation, Demonstration
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.data.io import load_json_files, load_lfd_configuration
from cairo_lfd.data.vectorization import vectorize_demonstration, get_observation_joint_vector
from cairo_lfd.data.alignment import DemonstrationAlignment
from cairo_lfd.data.processing import DataProcessingPipeline, RelativeKinematicsProcessor, RelativePositionProcessor, InContactProcessor, SphereOfInfluenceProcessor, WithinPerimeterProcessor
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.constraints.triggers import TriggerFactory
from cairo_lfd.core.lfd import LFD
from cairo_lfd.controllers.study_controllers import ACCLfDController


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
        '-i', '--input_directory', dest='input_directory', required=False,
        help='the directory from which to input prior demonstration .json files'
    )

    required.add_argument(
        '-o', '--output_directory', dest='output_directory', required=False,
        help='the directory to save the given subjects .json files data'
    )

    required.add_argument(
        '-t', '--task', dest='task', required=False,
        help='the name of the task being demonstrated'
    )

    required.add_argument(
        '-s', '--subject', dest='subject', required=False,
        help='the ID of the subject'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    ############################
    #  ROS Node Initialization #
    ############################
    print("Initializing node... ")
    rospy.init_node("ar4lfd")
    print("Getting robot state... ")
    robot_state = intera_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    robot_state.enable()

    ########################
    # Import Configuration #
    ########################
    config_filepath = args.config
    configs = load_lfd_configuration(config_filepath)

    #############################
    # Build Environment Objects #
    #############################
    items = ItemFactory(configs).generate_items()
    triggers = TriggerFactory(configs).generate_triggers()
    constraints = ConstraintFactory(configs).generate_constraints()
    # We only have just the one robot...for now.......
    environment = Environment(items=items['items'], robot=items['robots'][0], constraints=constraints, triggers=triggers)

    #####################################
    # Raw Demonstration Data Processors #
    #####################################
    # Build processors and process demonstrations to generate derivative data e.g. relative position.
    rk_processor = RelativeKinematicsProcessor(environment.get_item_ids(), environment.get_robot_id())
    ic_processor = InContactProcessor(environment.get_item_ids(), environment.get_robot_id(), .06, .5)
    soi_processor = SphereOfInfluenceProcessor(environment.get_item_ids(), environment.get_robot_id())
    rp_processor = RelativePositionProcessor(environment.get_item_ids(), environment.get_robot_id())
    wp_processor = WithinPerimeterProcessor(environment.get_item_ids(), environment.get_robot_id())
    processing_pipeline = DataProcessingPipeline([rk_processor, ic_processor, soi_processor, rp_processor, wp_processor])

    ###############################################
    # Configure the Sawyer Demonstration Recorder #
    ###############################################
    rec_settings = configs["settings"]["recording_settings"]
    recorder = SawyerDemonstrationRecorder(rec_settings, environment, processing_pipeline, publish_constraint_validity=True)
    rospy.on_shutdown(recorder.stop)

    ##############################################
    # Configure the Sawyer Demonstration Labeler #
    ##############################################
    label_settings = configs["settings"]["labeling_settings"]
    # Demonstration vectorizor that converts observations into state vector in desired space for DTW alignment.
    demo_vectorizor = partial(vectorize_demonstration, vectorizors=[get_observation_joint_vector])
    alignment = DemonstrationAlignment(demo_vectorizor)
    demo_labeler = SawyerDemonstrationLabeler(label_settings, alignment)

    #################################
    # Configure the LFD class model #
    #################################
    model_settings = configs["settings"]["modeling_settings"]
    moveit_interface = SawyerMoveitInterface()
    moveit_interface.set_velocity_scaling(.35)
    moveit_interface.set_acceleration_scaling(.25)
    moveit_interface.set_planner(str(model_settings["planner"]))
    lfd = LFD(configs, model_settings, moveit_interface)

    #########################
    # Import Initial Demos  #
    #########################
    lfd.build_environment()
    if args.input_directory is not None:

        prior_poor_demonstrations = load_json_files(args.input_directory + "/*.json")
        # Convert imported data into Demonstrations and Observations
        demonstrations = []
        for datum in prior_poor_demonstrations["data"]:
            observations = []
            for entry in datum:
                observations.append(Observation(entry))
            demonstrations.append(Demonstration(observations))
        if len(demonstrations) == 0:
            rospy.logwarn("No prior demonstration data to model!! You sure you're using the right experiment script?")
            return 0
        labeled_initial_demos = demo_labeler.label(demonstrations)
        lfd.build_keyframe_graph(labeled_initial_demos, model_settings.get("bandwidth", .025), )
        lfd.sample_keyframes(model_settings.get("number_of_samples", 50), automate_threshold=True)
    else:
        labeled_initial_demos = []
        lfd.build_keyframe_graph(labeled_initial_demos, model_settings.get("bandwidth", .025))


    #######################################
    # Set defaults for command line args  #
    #######################################

    if args.task is None:
        output_directory = ""
    else:
        output_directory = args.output_directory

    if args.task is None:
        task_name = "****"
    else:
        task_name = args.task

    if args.subject is None:
        subject = "####"
    else:
        subject = args.subject

    #########################
    # Run Study Controller  #
    #########################
    study = LfDController(acclfd, recorder, demo_labeler, labeled_initial_demos, output_directory, task_name, subject)
    study.run()


if __name__ == '__main__':
    main()