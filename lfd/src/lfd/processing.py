import numpy as np
from geometry_msgs.msg import Pose
from environment import Observation


def convert_data_to_pose(position, orientation):

    """
    Converts raw position and orientation data to a ROS message Pose object.

    Returns
    -------

    pose: geometry_msgs.msgs.Pose
        The Pose object
    """

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose


class SawyerSampleConverter(object):

    def __init__(self, interface):
        self.interface = interface

    def convert(self, sample, run_fk=True):
        """


        Parameters
        ----------


        Returns
        -------

        """
        if run_fk is True:
            pose = self.interface.get_end_effector_pose(sample.tolist())
            if pose is not None:
                sample = np.insert(sample, 0, pose.orientation.w, axis=0)
                sample = np.insert(sample, 0, pose.orientation.z, axis=0)
                sample = np.insert(sample, 0, pose.orientation.y, axis=0)
                sample = np.insert(sample, 0, pose.orientation.x, axis=0)
                sample = np.insert(sample, 0, pose.position.z, axis=0)
                sample = np.insert(sample, 0, pose.position.y, axis=0)

        normalize = np.sqrt(sample[3]**2 + sample[4]**2 +
                            sample[5]**2 + sample[6]**2)
        sample[3] = sample[3] / normalize
        sample[4] = sample[4] / normalize
        sample[5] = sample[5] / normalize
        sample[6] = sample[6] / normalize

        if len(sample) > 7:
            # If length > 7, we know there must be joint data, so creat Obs w/ joints.
            obsv = Observation.init_samples(sample[0:3], sample[3:7], sample[7:14])
        else:
            obsv = Observation.init_samples(sample[0:3], sample[3:7], None)
        return obsv

