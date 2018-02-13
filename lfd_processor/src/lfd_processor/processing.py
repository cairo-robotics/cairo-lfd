import math
import numpy as np
from geometry_msgs.msg import Pose


class DataProcessor(object):

    """
    Data prcoessing class with a variety of methods to support manipulating imported data.
    """

    def fixed_length_sampler(self, entries, fixed_length=250):

        """
        Takes a list and returns a list of a fixed length evenly pulled from the original list.

        Parameters
        ----------
        entries : list
            Original list to reduce to a list of fixed length.
        fixed_length : int
            The fixed length.

        Returns
        -------
        fl_entries : list
           A list of entries of fixed length, evenly distributed from the original list.
        """

        fl_entries = []
        for entry in entries:
            if len(entry) >= fixed_length:
                fl_entry = []
                length = float(len(entry))
                for i in range(fixed_length):
                    fl_entry.append(entry[int(math.ceil(i * length / fixed_length))])
                fl_entries.append(fl_entry)
            else:
                raise Exception("Fixed length is less than length of entry. \\"
                                "Try reducing the fixed length of a trajectory/keyframe")
        return fl_entries

    def convert_observation_dict_to_list(self,
                                         observation,
                                         key_order=["PoseX", "PoseY", "PoseZ", "OrienX",
                                                    "OrienY", "OrienZ", "OrienW", "time"]):

        """
        Takes an observation represented as a dictionary and converts it into a list.

        Parameters
        ----------
        observation : dict
            Dictionary representation of an observation.
        key_order : list of string
            A list of keys in the order that each element of the generated list will represent.

        Returns
        -------
        observation_list : list
           A list representation of the observation dict.
        """

        observation_list = []
        for key in key_order:
            observation_list.append(observation[key])
        return observation_list

    def to_np_array(self, sequence, type='f'):
        """
        Wrapper function converting a list to a numpy array.

        Parameters
        ----------
        sequence : list
            The list to convert.
        type : string
            The numpy type that the elements will be converted to in the array.

        Returns
        -------
        : numpy.ndarray
           Numpy array converted list.
        """
        return np.array(sequence, dtype=type)


class ObservationConverter(object):

    def generate_pose(self, object_state):
        pose = Pose()
        pose.position.x = object_state["PoseX"]
        pose.position.y = object_state["PoseY"]
        pose.position.z = object_state["PoseZ"]
        pose.orientation.x = object_state["OrienX"]
        pose.orientation.y = object_state["OrienY"]
        pose.orientation.z = object_state["OrienX"]
        pose.orientation.w = object_state["OrienW"]
        return pose
