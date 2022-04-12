
import rospy
import tf2_ros
import tf2_geometry_msgs
import time
import numpy as np
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform,  Pose, PoseStamped, Point, Quaternion, Vector3



class VRPNFixedTransform():
    def __init__(self, vrpn_world_frame="optitrack_world", global_world_frame="world", translation=Vector3(0, 0, 0), rotation=Quaternion(0, 0, 0, 1)):
        """

        """
        self.vrpn_world_frame = vrpn_world_frame
        self.global_world_frame = global_world_frame
        self._update_transform(translation, rotation)

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
        self._publish_transform()

    def _publish_transform(self):
        self.tf2_static_broadcaster.sendTransform(self.static_transform)


class ARPOLfDMiddleware():

    def __init__(self, joint_configuration_topic="/joint_configuration", playback_cmd_topic="/playback_cmd", joint_trajectory_topic="/joint_trajectory", translation=Vector3(0, 0, 0), rotation=Quaternion(0, 0, 0, 1)):
        """

        """
        self.join_configuration_topic = joint_configuration_topic
        self.playback_cmd_topic = playback_cmd_topic
        self.join_configuration_topic = joint_trajectory_topic

        self.vrpn_transformer = VRPNFixedTransform(translation, rotation)

        # Initialize Publishers
        self.joint_configuration_publisher = rospy.Publisher(joint_configuration_topic, String, queue_size=1)
        self.joint_trajectory_publisher = rospy.Publisher(joint_trajectory_topic, String, queue_size=1)
        
        # Initialize Subscribers
        self.replay_subscriber = rospy.Subscriber(
            joint_configuration_topic, String, queue_size=1)
        self.pose_optimization_subscriber = rospy.Subscriber()

    def publish_joint_configuration(self, msg):
        '''
        Once message is received on this topic, start process of displaying trajectory
        '''
        self.request_pub.publish(String("get_representation"))

        
        # transmit JSON for this transformed pose
        keyframe_msg = String()
        keyframe_msg.data = json.dumps(traj_pusher)
        self.hololens_pub.publish(keyframe_msg)

    def _constraint_cb(self, msg):
        w
        self.constraints_pub.publish(transformed_constraint_msg)