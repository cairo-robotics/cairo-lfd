#!/usr/bin/env python2.7
"""
DEPRECATED / NEEDS UPDATE
"""
import rospy
import argparse
import os
from lfd_processor.interfaces import Environment, import_configuration
from lfd_processor.items import RobotFactory, ConstraintFactory
from json import loads, dumps
from pprint import PrettyPrinter

if __name__ == "__main__":

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--filepath', dest='filepath', required=False,
        help='the filepath of the configuration file .config.json.'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    if args.filepath is not None:
        filepath = args.filepath
    else:
        dir = os.path.dirname(__file__)
        filepath = os.path.join(dir, "../toy_data/configurations/config.json")

    # The robot node sdk_join_recorder node must be initilized to make calls to reference intera_sdk
    # in the robot objects.
    print("Initializing node... ")
    rospy.init_node("sdk_joint_recorder")

    configs = import_configuration(filepath)
    robot_factory = RobotFactory(configs["robots"])
    constraint_factory = ConstraintFactory(configs["constraints"])

    robot = robot_factory.generate_robots()[0]
    constraints = constraint_factory.generate_constraints()

    # We only have just the one robot...for now.......
    environment = Environment(items=None, robot=robot, constraints=constraints)

    print
    print "Environment's robot information:"
    print
    pp = PrettyPrinter(indent=4)
    info = environment.get_robot_info()
    pp.pprint(loads(dumps(info)))

    print
    print "Environment's robot state:"
    print
    state = environment.get_robot_state()
    pp.pprint(loads(dumps(state)))

    print
    print "Environment's constraint information:"
    print
    for constraint in environment.constraints:
        print "Constraint Id: {}".format(constraint.id)
        print "Constraint Item: {}".format(constraint.item_id)
        print
