#!/usr/bin/env python

from cairo_lfd.services.trigger_service import ConstraintWebTriggerService
import rospy

if __name__ == "__main__":
    rospy.init_node("constraint_trigger_server")
    cts = ConstraintWebTriggerService(service_name="constraint_trigger_service")
    cts.start_server()