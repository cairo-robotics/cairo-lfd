#!/usr/bin/env python2

import rospy
import signal
import argparse
import intera_interface
from intera_interface import CHECK_VERSION
from intera_motion_interface import InteractionOptions, InteractionPublisher
from cairo_lfd.core.record import SawyerRecorder
from cairo_lfd.core.environment import Environment, import_configuration
from cairo_lfd.core.items import ItemFactory
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.constraints.triggers import TriggerFactory
from cairo_lfd.modeling.analysis import ConstraintAnalyzer
from cairo_lfd.data.io import DataExporter


class GracefulQuit:
    kill = False

    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        self.kill_now = True


def main():
    """
    Demonstration Recorder

    Record a series of demonstrations.
    """
    rospy.init_node("trigger_test")
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-c', '--config', dest='config', required=True,
        help='the file path of the demonstration '
    )
    args = parser.parse_args(rospy.myargv()[1:])

    config_filepath = args.config
    configs = import_configuration(config_filepath)

    triggers = TriggerFactory(configs).generate_triggers()

    for trigger in triggers:
        print(type(trigger))

    config_filepath = args.config
    configs = import_configuration(config_filepath)

    items = ItemFactory(configs).generate_items()
    triggers = TriggerFactory(configs).generate_triggers()
    constraints = ConstraintFactory(configs).generate_constraints()
    # We only have just the one robot...for now.......
    environment = Environment(items=items['items'], robot=items['robots'][0], constraints=constraints, triggers=triggers)
    quit = GracefulQuit()
    while not quit.kill:
        rospy.sleep(2)
        print(environment.check_constraint_triggers())

if __name__ == '__main__':
    main()
