#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
import json
import time
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped, Transform, Pose, PoseStamped, Point, Quaternion, Vector3
from tf.transformations import translation_matrix, quaternion_matrix, concatenate_matrices, inverse_matrix, \
    translation_from_matrix, quaternion_from_matrix
'''
Contains a class for transforming from a fixed coordinate system to a HoloLens with a known origin point

Code modified from Dan Koral's arvr_device package: https://github.com/cu-ironlab/ros_arvr_device_and_robot_management
'''

class FixedTransformManager(object):
    def __init__(self, name, origin_translation, origin_rotation, selector_matrix):
        """
        :type name: str
        :type origin_translation: Vector3
        :type origin_rotation: Quaternion
        :type selector_matrix: list
        :param name:
        :param origin_translation:
        :param origin_rotation:
        :param selector_matrix:
        """
        self.name = name
        self.selector_matrix = np.array(selector_matrix)
        self.last_lookup = time.time()
        self.frame_transform = None
        self._update_transform(origin_translation, origin_rotation)

        # TF Specific Stuff
        self.tf2_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        self._publish_transform()

    def _update_transform(self, translation, rotation):
        """
        :type translation: Point
        :type rotation: Quaternion
        :param translation:
        :param rotation:
        """
        self.static_transform = TransformStamped()
        self.static_transform.header.frame_id = "world"
        self.static_transform.child_frame_id = self.name
        self.static_transform.transform = Transform(translation, rotation)
        self.static_transform.header.stamp = rospy.Time.now()

    def _publish_transform(self):
        self.tf2_static_broadcaster.sendTransform(self.static_transform)

    def world_to_hololens(self, pose):
        """
        :type pose: Pose
        :param pose: point in world coordinates to be transformed to HoloLens space
        """

        #lookup transform between world and this device if it has been longer than one second since last lookup
        if(self.last_lookup is None or time.time() - self.last_lookup > 1):
            self.frame_transform = self.tf2_buffer.lookup_transform(self.name, "world", rospy.Time(0), rospy.Duration(1.0))
            self.last_lookup = time.time()

        #apply transform to point
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, self.frame_transform)

        #switch coordinate axes
        device_pose = world_to_arvr(transformed_pose, self.selector_matrix)
        return device_pose



'''
Helper functions modified from https://github.com/cu-ironlab/ros_arvr_device_and_robot_management/blob/master/src/arvr_utility/msg_transformations.py
'''
def arvr_to_world(msg, transformation_matrix):
    """
    This is a helper function to transform a arvr coordinate to the world space when given the
    selector transformation matrix that is specific to that arvr device.
    :type msg: TransformStamped
    :type transformation_matrix: np.ndarray
    :param msg:
    :param transformation_matrix:
    :return:
    """
    translation_vector = np.array([msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                    1]).transpose()
    quaternion_vector = np.array([msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w]).transpose()

    translation_vector = transformation_matrix.dot(translation_vector)
    quaternion_vector = transformation_matrix.dot(quaternion_vector)

    p = PoseStamped()
    p.header = msg.header
    p.pose.position.x = translation_vector[0]
    p.pose.position.y = translation_vector[1]
    p.pose.position.z = translation_vector[2]
    p.pose.orientation.x = quaternion_vector[0]
    p.pose.orientation.y = quaternion_vector[1]
    p.pose.orientation.z = quaternion_vector[2]
    p.pose.orientation.w = quaternion_vector[3]

    return p


def world_to_arvr(msg, transformation_matrix):
    """
    This is a helper function to transform a world coordinate to an arvr coordinate when given the
    selector transformation matrix that is specific to that arvr device. To convert it back in the
    opposite direction, you must use the inverse of the transformation_matrix and then do the normal
    swaps in the arvr_to_world() function.
    :type msg: PoseStamped
    :type transformation_matrix: np.ndarray
    :param msg:
    :param transformation_matrix:
    :return:
    """
    inverse = inverse_matrix(transformation_matrix)
    return arvr_to_world(msg, inverse)


class Middleman(object):
    def __init__(self, command_topic, request_topic, traj_representation_topic, hololens_pub_topic):
        """
        :type command_topic: str
        :type request_topic: str
        :type traj_representation_topic: str
        :type hololens_pub_topic: str
        :param command_topic:
        :param request_topic:
        :param traj_representation_topic:
        :param hololens_pub_topic:
        """

        #Create transform manager (position and axes hardcoded for now)
        self.transform_manager = FixedTransformManager( "hololens", Vector3(1.0, 0.0, -0.2632),
                                Quaternion(0.0, 0.0, 1.0, 0.0), [[0, 0, 1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, -1]] )

        #Initialize Publishers
        self.request_pub = rospy.Publisher(request_topic, String, queue_size=1)
        self.hololens_pub = rospy.Publisher(hololens_pub_topic, String, queue_size=1)

        #Initialize Subscribers
        self.command_sub = rospy.Subscriber(command_topic, String, self._command_cb)
        self.trajectory_sub = rospy.Subscriber(traj_representation_topic, String, self._trajectory_cb)

    def _command_cb(self, msg):
        '''
        Once message is received on this topic, start process of displaying trajectory
        '''
        self.request_pub.publish(String("get_representation"))


    def _trajectory_cb(self, msg):

        #Clear out any visualizations currently present first
        self.hololens_pub.publish(String("CLEAR"))
        time.sleep(1.0)

        #Individually publish each keyframe to the HoloLens
        traj_object = json.loads(msg.data)
        traj_pusher = {}
        traj_pusher["trajectory"] = []
        for point_object in traj_object["point_array"]:
            #Update position and orientation
            pos = point_object["robot"]["position"]
            quat = point_object["robot"]["orientation"]
            updated_pose = self.transform_manager.world_to_hololens(Pose(Point(pos[0], pos[1], pos[2]),Quaternion(quat[0], quat[1], quat[2], quat[3])))

            #update JSON with transformed pose
            json_object = {}
            json_object["keyframe_id"] = point_object["keyframe_id"]
            json_object["applied_constraints"] = point_object["applied_constraints"]
            json_object["robot"] = {}
            json_object["robot"]["position"] = [updated_pose.pose.position.x, updated_pose.pose.position.y, updated_pose.pose.position.z]
            json_object["robot"]["orientation"] = [updated_pose.pose.orientation.x, updated_pose.pose.orientation.y, updated_pose.pose.orientation.z, updated_pose.pose.orientation.w]

            traj_pusher["trajectory"].append(json_object)
        #transmit JSON for this transformed pose
        keyframe_msg = String()
        keyframe_msg.data = json.dumps(traj_pusher)
        self.hololens_pub.publish(keyframe_msg)


def main():
    rospy.init_node("middleman")
    mm = Middleman("start_ar", "/cairo_lfd/model_commands", "/cairo_lfd/lfd_representation", "/constraint_manager")
    rospy.spin()


if __name__ == '__main__':
    main()
