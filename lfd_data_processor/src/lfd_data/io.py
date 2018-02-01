import glob
import errno
import csv
import json
import codecs
from collections import OrderedDict


class DataExporter:
    """
    Data exporint class with a variety of methods to support importing trajectory/observation data from csv, json etc.
    """

    def export_to_json(self, path, data):
        """
        Exports dictioanry data to a .json file.

        Parameters
        ----------
        path : string
            Path of directory to save .json file.
        dict_data : dict/JSON serializable type
            Data to be serialized to json.
        """
        with open(path, 'w') as f:
            json.dump(data, f,  indent=4, sort_keys=True)


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

    def import_json_to_dict(self, path):
        """
        Import trajectories stored as .json files into a Ordered Dictionary. In this case, each file represents
        a single trajectory/demonstration.

        This method expects a directory path and will automatically import all files with an approporaite .json
        file signature.

        Parameters
        ----------
        path : string
            Path of directory containing the .csv files.

        Returns
        -------
        entries : OrderedDict
            Dictionary of trajectories. Trajectories themselves are dictionaries of observations.
        """
        data = json.load(open('config.json'), object_pairs_hook=OrderedDict)

        entries = OrderedDict()
        entries["trajectories"] = []
        files = glob.glob(path)
        for name in files:
            try:
                with codecs.open(name, "r", 'utf-8') as f:
                    trajectories = json.load(f, object_pairs_hook=OrderedDict)
                    entries['trajectories'].append(trajectories)
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