#!/usr/bin/env python
from cairo_lfd.modeling.analysis import ConstraintAnalyzer
from moveit_msgs.srv import CustomCost, CustomCostResponse
import rospy


class CustomCostService():
    """
    Class that creates a ROS service to handle incoming calls to calculate
    OMPL costs.

    Attributes
    ----------
    service_name : str
        The ROS Service proxy object
    """
    def __init__(self, service_name):
        """
        Parameters
        ----------
        service_name : str
            The ROS Service proxy object
        """
        self.environment = environment
        self.service_name = service_name
        self.constraints = constraints

    def callback(self, robot_state):
        rospy.logerr(self.constraint_states)

    def start_server(self):
        """
        Initiates/starts the Constraint Web Trigger service
        """
        rospy.init_node('custom_cost_server')
        rospy.Service('custom_cost', CustomCost, self.callback)
        s = rospy.Service(self.service_name, ConstraintWebTrigger, self.get_state)
        rospy.loginfo("{} service running...".format(self.service_name))
        rospy.spin()

    def 