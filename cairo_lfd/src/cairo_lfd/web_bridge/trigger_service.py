from functools import partial

import rospy
from std_msgs.msg import Bool
from cairo_lfd.srv import ConstraintWebTrigger, ConstraintWebTriggerRequest, ConstraintWebTriggerResponse


class ConstraintWebTriggerClient():
    def __init__(self, service_name):
        self.ns = service_name
        # TODO reconnect logic since we're using persistent connection
        self.service = rospy.ServiceProxy(self.ns, ConstraintWebTrigger, persistent=True)
        rospy.loginfo("Connecting to Constraint Trigger service.")
        try:
            rospy.wait_for_service(self.ns, 20)
            rospy.loginfo("Connected to Constraint Trigger service")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

    def close(self):
        self.service.close()

    def call(self, constraint_name):

        req = ConstraintWebTriggerRequest()
        req.constraint_name = constraint_name

        try:
            rospy.wait_for_service(self.ns, 5.0)
            resp = self.service(req)
            return resp.status
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Web trigger service call failed: %s" % (e,))
            return None

        if resp.error.error_string is not None:
            rospy.logwarn("Web trigger service call failed: %s" % (e,))
            return None


class ConstraintWebTriggerService():
    """
    Class that creates a ROS service to handle incoming calls to calculate
    transformations from one frame to another. 

    Attributes
    ----------
    service_name : str
        The ROS Service proxy object
    """
    def __init__(self, service_name, constraint_states=None):
        """
        Parameters
        ----------
        service_name : str
            The ROS Service proxy object
        """
        self.constraint_states = constraint_states if constraint_states is not None else {
            "orientation_constraint": False,
            "height_constraint": False,
            "over_under_constraint": False,
            "perimeter_constraint": False
        }
        self.service_name = service_name

    def callback(self, data, topic):
        self.constraint_states[topic] = data.data
        rospy.logerr(self.constraint_states)

    def build_subscribers(self):
        subscribers = []
        for topic in self.constraint_states.keys():
            subscribers.append(rospy.Subscriber(topic, Bool, self.callback, callback_args=topic))
        return subscribers

    def get_state(self, req):
        """
        Function to lookup transform given a TransformLookupRequest

        Parameters
        ----------
        req : ConstraintTriggerServiceRequest
            The request for the ConstraintTriggerService

        Returns
        -------
        res : ConstraintTriggerServiceResponse
            The response indicting the triggered status for the requested constraint name.
        """
        resp = ConstraintWebTriggerResponse()
        resp.status = self.constraint_states[req.constraint_name]
        return resp     

    def start_server(self):
        """
        Initiates/starts the Constraint Web Trigger service
        """
        rospy.loginfo("Server started")
        subscribers = self.build_subscribers()
        s = rospy.Service(self.service_name, ConstraintWebTrigger, self.get_state)
        rospy.loginfo("{} service running...".format(self.service_name))
        rospy.spin()