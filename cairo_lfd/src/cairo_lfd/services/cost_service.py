import rospy


from cairo_lfd.modeling.analysis import ConstraintAnalyzer
from cairo_lfd.data.conversion import SawyerSampleConverter

from cairo_lfd_msgs.msg import AppliedConstraints

from cairo_robot_interface.moveit_interface import SawyerMoveitInterface

try:
    from moveit_msgs.srv import CustomCost
except ImportError as e:
    rospy.logerr(e)
    return 0


class CustomCostService():
    """
    Class that creates a ROS service to handle incoming calls to calculate
    OMPL costs.
    """
    def __init__(self, environment, constraints_topic='/lfd/applied_constraints', cost_service_name='custom_cost'):
        self.service_name = cost_service_name
        self.constraints_topic = constraints_topic
        self.environment = environment
        self.applied_constraints = []
        self.analyzer = ConstraintAnalyzer(environment)
        self.converter = SawyerSampleConverter(SawyerMoveitInterface())

    def cost_callback(self, custom_cost_request):
        rospy.loginfo(custom_cost_request)
        joints = custom_cost_request.state
        observation = self.converter.convert(joints, run_fk=True, normalize_quaternion=True)
        valid_set, valid_ids = self.analyzer.evaluate(self.applied_constraints, observation)
        if valid_set:
            return 0
        else:
            # inifinte cost
            return 100000000

    def set_constraints_callback(self, data):
        self.applied_constraints = data.constraints

    def start_server(self):
        """
        Initiates/starts the Custom Cost function service and the current applied constraints subscriber
        """
        rospy.init_node('custom_cost_server')
        rospy.Service('custom_cost', CustomCost, self.callback)
        rospy.Service(self.service_name, CustomCost, self.cost_callback)
        rospy.loginfo("{} service running...".format(self.service_name))

        rospy.Subscriber(self.constraints_topic, AppliedConstraints, self.set_constraints_callback)
        rospy.spin()
