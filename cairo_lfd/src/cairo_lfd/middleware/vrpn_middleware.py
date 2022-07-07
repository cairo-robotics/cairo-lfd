
import rospy
import tf2_ros

import numpy as np

from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Vector3



class VRPNFixedTransform():
    def __init__(self, vrpn_world_frame="optitrack_world", global_world_frame="world", translation=Vector3(0, 0, 0), rotation=Quaternion(0, 0, 0, 1)):
        self.vrpn_world_frame = vrpn_world_frame
        self.global_world_frame = global_world_frame
        self._update_transform(translation, rotation)

        # TF Specific Stuff
        self.tf2_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        self._publish_transform()

    def update_transform(self, translation, rotation):
        self.static_transform = TransformStamped()
        self.static_transform.header.frame_id = self.global_world_frame
        self.static_transform.child_frame_id = self.vrpn_world_frame
        self.static_transform.transform = Transform(translation, rotation)
        self.static_transform.header.stamp = rospy.Time.now()
        self._publish_transform()

    def _publish_transform(self):
        self.tf2_static_broadcaster.sendTransform(self.static_transform)


