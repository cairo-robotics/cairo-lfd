import rospy


from cairo_lfd.modeling.analysis import ConstraintAnalyzer
from cairo_lfd.core.environment import Observation
from cairo_lfd.data.conversion import SawyerSampleConverter

from cairo_lfd_msgs.msg import KeyframeConstraints

from cairo_robot_interface.

try:
    from moveit_msgs.srv import CustomCost, CustomCostResponse
except ImportError as e:
    rospy.logerr(e)
    return 0


class CustomCostService():
    """
    Class that creates a ROS service to handle incoming calls to calculate
    OMPL costs.

    Attributes
    ----------
    service_name : str
        The ROS Service proxy object
    """
    def __init__(self, service_name, environment):
        """
        Parameters
        ----------
        service_name : str
            The ROS Service proxy object
        """
        self.service_name = service_name
        self.environment = environment
        self.applied_constraints = []
        self.analyzer = ConstraintAnalyzer(environment)

    def cost_callback(self, custom_cost_request):
        joints = custom_cost_request.state


    def set_constraints_callback(self, data):
        self.applied_constraints = data.constraints

    def start_server(self):
        """
        Initiates/starts the Constraint Web Trigger service
        """
        rospy.init_node('custom_cost_server')
        rospy.Service('custom_cost', CustomCost, self.callback)
        s = rospy.Service(self.service_name, CustomCost, self.cost_callback)
        rospy.loginfo("{} service running...".format(self.service_name))

        rospy.Subscriber('/lfd/applied_constraints', KeyframeConstraints, self.set_constraints_callback)
        rospy.spin()

    def 