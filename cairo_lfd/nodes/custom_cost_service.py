#!/usr/bin/env python
import argparse
import rospy

from cairo_lfd.services.cost_service import CustomCostService
from cairo_lfd.data.io import import_configuration
from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.core.environment import Environment
from cairo_lfd.core.items import ItemFactory


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-c', '--config', dest='config', required=True,
        help='the file path of configuration config.json file '
    )

    args = parser.parse_args(rospy.myargv()[1:])

    config_filepath = args.config
    configs = import_configuration(config_filepath)

    items = ItemFactory(configs).generate_items()
    constraints = ConstraintFactory(configs).generate_constraints()
    # We only have just the one robot...for now.......
    environment = Environment(items=items['items'], robot=items['robots']
                              [0], constraints=constraints, triggers=None)

    rospy.init_node("constraint_trigger_server")
    ccs = CustomCostService(environment, constraints_topic='/lfd/applied_constraints', cost_service_name='custom_cost')
    ccs.start_server()


if __name__ == '__main__':
    main()
