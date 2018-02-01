import rospy
import intera_interface
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Pose


def convert_data_to_pose(position, orientation):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        return pose


class AbstractItem(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def get_state(self):
        pass

    @abstractmethod
    def get_info(self):
        pass


class SawyerRobot(AbstractItem):

    def __init__(self, robot_id, upright_pose):
        self.id = robot_id
        self.upright_pose = upright_pose
        self._limb = intera_interface.Limb("right")
        self._cuff = intera_interface.Cuff("right")
        self._navigator = intera_interface.Navigator()
        try:
            self._gripper = intera_interface.Gripper("right")
            rospy.loginfo("Electric gripper detected.")
            if self._gripper.has_error():
                rospy.loginfo("Gripper error...rebooting.")
                self._gripper.reboot()
            if not self._gripper.is_calibrated():
                rospy.loginfo("Calibrating gripper.")
                self._gripper.calibrate()
        except Exception as e:
            self._gripper = None
            rospy.loginfo("No electric gripper detected.")

    def get_state(self):
        """
        Returns state of robot:
        {
            id: robot_id
            position: [x, y ,z],
            orientation: [x, y, z, w]
            joints: [j0, j1, j2, j3, j4, j5, j6],
            gripper: value
        }
        """
        state = {}
        joints = self._limb.joint_names()
        pose = self._limb.endpoint_pose()
        state["id"] = self.id
        state['position'] = pose['position']
        state['orientation'] = pose['orientation']
        state['gripper'] = self._gripper.get_position()
        state['joints'] = [self._limb.joint_angle(j) for j in joints]
        return state

    def get_info(self):
        return {
                    "id": self.id,
                    "upright_pose": self.upright_pose
               }
