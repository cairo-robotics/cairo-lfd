import rospy
import intera_interface


class SawyerRobot(object):

    def __init__(self, robot_id, upright_position):
        self.id = robot_id
        self.upright_position = upright_position
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
            position: [x, y ,z],
            orientation: [x, y, z, w]
            joints: [j0, j1, j2, j3, j4, j5, j6],
            gripper: value
        }
        """
        state = {}
        joints = self._limb.joint_names()
        pose = self._limb.endpoint_pose()
        state['position'] = pose['position']
        state['orientation'] = pose['orientation']
        state['gripper'] = self._gripper.get_position()
        state['joints'] = [self._limb.joint_angle(j) for j in joints]
        return state
