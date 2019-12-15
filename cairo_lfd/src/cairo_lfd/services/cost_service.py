import rospy


from cairo_lfd.modeling.analysis import ConstraintAnalyzer
from cairo_lfd.data.conversion import SawyerSampleConverter

from cairo_lfd_msgs.msg import AppliedConstraints

from robot_interface.moveit_interface import SawyerMoveitInterface


from moveit_msgs.srv import CustomCost, CustomCostResponse


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
        self.interface = SawyerMoveitInterface()
        self.analyzer = ConstraintAnalyzer(environment)
        self.converter = SawyerSampleConverter(self.interface)

    def cost_callback(self, custom_cost_request):

        joints = list(custom_cost_request.state)
        observation = self.converter.convert(joints, run_fk=True)
        constraints = [c for c in self.environment.constraints if c.id in self.applied_constraints]
        valid_set, valid_ids = self.analyzer.evaluate(constraints, observation)

        response = CustomCostResponse()
        if valid_set:
            # negative infinity
            response.cost = 2
        else:
            # infinity
            response.cost = 1
        return response

    def set_constraints_callback(self, data):
        rospy.loginfo("Applied constraint set: {}".format(self.applied_constraints))
        self.applied_constraints = data.constraints

    def start_server(self):
        """
        Initiates/starts the Custom Cost function service and the current applied constraints subscriber
        """
        rospy.Service(self.service_name, CustomCost, self.cost_callback)
        rospy.loginfo("{} service running...".format(self.service_name))

        rospy.Subscriber(self.constraints_topic, AppliedConstraints, self.set_constraints_callback)
        rospy.spin()
