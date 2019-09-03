#!/usr/bin/env python

from cairo_lfd.web_bridge.constraint_trigger_service import ConstraintTriggerService
import rospy

if __name__ == "__main__":
    rospy.init_node("constraint_trigger_server")
    cts = ConstraintTriggerService(service_name="constraint_trigger_service")
    cts.start_server()