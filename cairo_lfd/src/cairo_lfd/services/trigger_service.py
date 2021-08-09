from functools import partial

import rospy
from std_msgs.msg import Int8MultiArray
from cairo_lfd_msgs.srv import ConstraintWebTrigger, ConstraintWebTriggerRequest, ConstraintWebTriggerResponse


class ConstraintWebTriggerClient():
    def __init__(self, service_name):
        self.ns = service_name
        # TODO reconnect logic since we're using persistent connection
        self.service = rospy.ServiceProxy(self.ns, ConstraintWebTrigger, persistent=True)
        try:
            rospy.wait_for_service(self.ns, 20)
        except rospy.ServiceException as e:
            rospy.logerr("Service startup failed: %s" % (e,))
        except rospy.ROSException as e:
            rospy.logerr("General ROS exception: %s" % (e,))

    def close(self):
        self.service.close()

    def call(self, constraint_id):

        req = ConstraintWebTriggerRequest()
        req.constraint_id = constraint_id

        try:
            rospy.wait_for_service(self.ns, 5.0)
            resp = self.service(req)
            if resp.error.error_string is not None:
                rospy.logwarn("Web trigger service call failed: %s" % (e,))
                return None
            return resp.status
        except rospy.ServiceException as e:
            rospy.logerr("Web trigger service call failed: %s" % (e,))
            return None
        except rospy.ROSException as e:
            rospy.logerr("Ros Exception, service call failed: %s" % (e,))
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
        self.triggered_constraints = []
        self.service_name = service_name

    def triggered_callback(self, msg):
        self.triggered_constraints = msg.data

    def get_state(self, req):
        """
        Function to whether or not a given constraint has been triggered via the web-based interface.

        Parameters
        ----------
        req : ConstraintTriggerServiceRequest
            The request for the ConstraintTriggerService

        Returns
        -------
        res : ConstraintTriggerServiceResponse
            The response indicting the triggered status for the requested constraint id.
        """
        resp = ConstraintWebTriggerResponse()
        print(req.constraint_id, self.triggered_constraints)
        resp.status = True if req.constraint_id in self.triggered_constraints else False
        return resp

    def start_server(self):
        """
        Initiates/starts the Constraint Web Trigger service
        """
        subscribers = rospy.Subscriber("cairo_lfd/triggered_constraints", Int8MultiArray, self.triggered_callback)
        s = rospy.Service(self.service_name, ConstraintWebTrigger, self.get_state)
        rospy.loginfo("{} service running...".format(self.service_name))
        rospy.spin()