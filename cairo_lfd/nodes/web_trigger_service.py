#!/usr/bin/env python3

import rospy
from cairo_lfd.services.trigger_service import ConstraintWebTriggerService


def main():
    rospy.init_node("constraint_trigger_server")
    cts = ConstraintWebTriggerService(service_name="constraint_trigger_service")
    cts.start_server()

if __name__ == "__main__":
    main()
