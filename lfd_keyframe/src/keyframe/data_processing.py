import glob
import errno
import csv
import math
import json
import numpy as np
import codecs
from collections import OrderedDict


class DataImporter:

    """
    Data importing class with a variety of methods to support importing trajectory/observation data from csv, json etc.
    """

    def import_csv_to_list(self, path, exclude_header=True):
        """
        Import trajectories stored as .csv files into a list of trajectories. In this case, each file represents
        a single trajectory/demonstration.

        This method expects a directory path and will automatically import all files with an approporaite .csv
        file signature.

        Parameters
        ----------
        path : string
            Path of directory containing the .csv files.
        exclude_header : bool
            If true, first entry for each trajectory will be removed i.e. ignores the header.

        Returns
        -------
        entries : list
            List of trajectories. Trajectories themselves are lists of observations (rows of the .csv file).
        """
        entries = []
        files = glob.glob(path)
        for name in files:
            try:
                with codecs.open(name, "r", 'utf-8') as f:
                    reader = csv.reader(f)
                    trajectory = list(reader)
                    if exclude_header:
                        trajectory.pop(0)
                    entries.append(trajectory)
            except IOError as exc:
                if exc.errno != errno.EISDIR:
                    raise  # Propagate other kinds of IOError.
        return entries

    def import_csv_to_dict(self, path):
        """
        Import trajectories stored as .csv files into a Ordered Dictionary. In this case, each file represents
        a single trajectory/demonstration.

        This method expects a directory path and will automatically import all files with an approporaite .csv
        file signature.

        Parameters
        ----------
        path : string
            Path of directory containing the .csv files.

        Returns
        -------
        entries : OrderedDict
            Dictionary of trajectories. Trajectories themselves are dictionaries of observations (rows of the
            .csv file).
        """
        entries = OrderedDict()
        entries["trajectories"] = []
        files = glob.glob(path)
        for name in files:
            try:
                with codecs.open(name, "r", 'utf-8') as f:
                    reader = csv.DictReader(f)
                    trajectory = {}
                    trajectory["name"] = name
                    trajectory["observations"] = []
                    for row in reader:
                        trajectory["observations"].append(row)
                    entries['trajectories'].append(trajectory)
            except IOError as exc:
                if exc.errno != errno.EISDIR:
                    raise  # Propagate other kinds of IOError.
        return entries

    def load_json_files(self, path):
        """
        Import JSON file as a Python dictionary.

        Parameters
        ----------
        path : string
            Path of directory containing the .csv files.

        Returns
        -------
        entries : dict
            Dictionary representation of the JSON file.
        """
        with open(path, 'r') as f:
            datastore = json.load(f)
            return datastore


class DataProcessor:

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

