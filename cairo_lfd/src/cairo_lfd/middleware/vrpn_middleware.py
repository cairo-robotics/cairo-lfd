
import rospy
import tf2_ros
import tf2_geometry_msgs
import time
import numpy as np
from sensor_msgs import JointState
from geometry_msgs.msg import TransformStamped, Transform,  Pose, PoseStamped, Point, Quaternion, Vector3



class VRPNFixedTransform():
    def __init__(self, vrpn_world_frame="optitrack_world", global_world_frame="world", translation=Vector3(0, 0, 0), rotation=Quaternion(0, 0, 0, 1)):
        """

        """
        self.vrpn_world_frame = vrpn_world_frame
        self.global_world_frame = global_world_frame
        self._update_transform(translation, Quaternion)

        # TF Specific Stuff
        self.tf2_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        self._publish_transform()

    def update_transform(self, translation, rotation):
        """
        """
        self.static_transform = TransformStamped()
        self.static_transform.header.frame_id = self.global_world_frame
        self.static_transform.child_frame_id = self.vrpn_world_frame
        self.static_transform.transform = Transform(translation, rotation)
        self.static_transform.header.stamp = rospy.Time.now()

    def _publish_transform(self):
        self.tf2_static_broadcaster.sendTransform(self.static_transform)


class ARPOLfDMiddleware(object):

    def __init__(self, command_topic, joint_configuration_topic, traj_representation_topic, hololens_pub_topic, cons):
        """

        """

        # Create transform manager (position and axes hardcoded for now)
        self.transform_manager = VRPNFixedTransform(Vector3(0.975, -0.09, -0.27),  Quaternion(0.0, 0.0, 1.0, 0.0))

        # Initialize Publishers
        self.joint_state_publisher = rospy.Publisher(joint_configuration_topic, JointState, queue_size=1)
        self.hololens_pub = rospy.Publisher(
            joint_configuration_topic, JointState, queue_size=1)

        # Initialize Subscribers

    def _command_cb(self, msg):
        '''
        Once message is received on this topic, start process of displaying trajectory
        '''
        self.request_pub.publish(String("get_representation"))

        
        # transmit JSON for this transformed pose
        keyframe_msg = String()
        keyframe_msg.data = json.dumps(traj_pusher)
        self.hololens_pub.publish(keyframe_msg)

    def _constraint_cb(self, msg):
        constraint_msg = json.loads(msg.data)
        for constraint in constraint_msg["constraints"]:
            if(constraint["className"] == "HeightConstraintAbove" or constraint["className"] == "HeightConstraintBelow"):
                # Transform height constraint
                updated_pose = self.transform_manager.hololens_to_world(Pose(
                    Point(0, constraint["args"][0], 0), Quaternion(0, 0, 0, 1)))
                constraint["args"][0] = updated_pose.pose.position.z
            elif(constraint["className"] == "UprightConstraint"):
                # Transform upright constraint
                updated_pose = self.transform_manager.hololens_to_world(Pose(
                    Point(0, 0, 0), Quaternion(constraint["args"][0], constraint["args"][1],
                    constraint["args"][2], constraint["args"][3])))
                constraint["args"][0] = updated_pose.pose.orientation.x
                constraint["args"][1] = updated_pose.pose.orientation.y
                constraint["args"][2] = updated_pose.pose.orientation.z
                constraint["args"][3] = updated_pose.pose.orientation.w
            elif(constraint["className"] == "OverUnderConstraint"):
                # Transform over-under constraint
                updated_pose = self.transform_manager.hololens_to_world(Pose(
                    Point(constraint["args"][0], constraint["args"][1], constraint["args"][2]), 
                    Quaternion(0, 0, 0, 1)))
                constraint["args"][0] = updated_pose.pose.position.x
                constraint["args"][1] = updated_pose.pose.position.y
                #Subtract half a meter to make up for hololens visualization. TODO: fix this on HoloLens side
                constraint["args"][2] = updated_pose.pose.position.z - 0.5
            else:
                # Unsupported constraint type
                rospy.logwarn("Unsupported constraint type passed to transformer, being passed on as-is")

        transformed_constraint_msg = String()
        transformed_constraint_msg.data = json.dumps(constraint_msg)
        self.constraints_pub.publish(transformed_constraint_msg)