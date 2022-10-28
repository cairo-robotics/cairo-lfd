"""
The io.py module contains a classes that contain methods for importing and exporting data.
"""
import glob
import errno
import csv
import json
import codecs
from collections import OrderedDict


class ConfigurationError(Exception):

    def __init__(self, value):
        self.value = value

    def __str__(self):
        return(repr(self.value))


def load_lfd_configuration(filepath):
    """
    Wrapper function around json.load() to import a config.json file used to inform the Environment object.
    """
    with open(filepath) as json_data:
        configs = json.load(json_data, object_pairs_hook=OrderedDict)
        if "constraints" not in configs:
            raise ConfigurationError(
                "config.json file must contain the 'constraints' key, even if its value is an empty list.")
        if "items" not in configs:
            raise ConfigurationError(
                "config.json file must contain the 'items' key, even if its value is an empty list.")
        if "robots" not in configs:
            raise ConfigurationError(
                "config.json file must contain the 'robots' key, even if its value is an empty list.")
        if "triggers" not in configs:
            raise ConfigurationError(
                "config.json file must contain the 'triggers' key, even if its value is an empty list.")
        validate_unique_item_ids(configs['robots'], configs['items'])
        return configs


def export_to_json(path, data):
    """
    Exports dictionary data to a .json file.

    Parameters
    ----------
    path : string
        Path of directory to save .json file.
    dict_data : dict/JSON serializable type
        Data to be serialized to json.
    """
    with open(path, 'w') as f:
        json.dump(data, f, indent=4)


def import_csv_to_list(path, exclude_header=True):
    """
    Import trajectories stored as .csv files into a list of trajectories. In this case, each file represents
    a single trajectory/demonstration.

    This method expects a directory path and will automatically import all files with an appropriate .csv
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


def import_csv_to_dict(path):
    """
    Import trajectories stored as .csv files into a Ordered Dictionary. In this case, each file represents
    a single trajectory/demonstration.

    This method expects a directory path and will automatically import all files with an appropriate .csv
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


def import_trajectory_json_to_dict(path):
    """
    Import trajectories stored as .json files into a Ordered Dictionary. In this case, each file represents
    a single trajectory/demonstration.

    This method expects a directory path and will automatically import all files with an appropriate .json
    file signature.

    Parameters
    ----------
    path : string
        Path of directory containing the .json files.

    Returns
    -------
    entries : OrderedDict
        Dictionary of trajectories. Trajectories themselves are dictionaries of observations.
    """

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


def load_json_files(path):
    """
    Import JSON files as a Python dictionary from .json files in the directory signified by the path..

    Parameters
    ----------
    path : string
        Path of directory containing the ..json files.

    Returns
    -------
    entries : dict
        Dictionary representation of the JSON file.
    """

    entries = OrderedDict()
    entries["data"] = []
    files = glob.glob(path)
    for name in files:
        try:
            with codecs.open(name, "r", 'utf-8') as f:
                file_data = json.load(f, object_pairs_hook=OrderedDict)
                entries["data"].append(file_data)
        except IOError as exc:
            if exc.errno != errno.EISDIR:
                raise  # Propagate other kinds of IOError.
    return entries


def load_json_file(path):
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


def validate_unique_item_ids(robots, items):
    print(robots)
    print(items)
    ids = []
    for robot in robots:
        if robot['init_args']['item_id'] in ids:
            raise ValueError("Robots and Items must have unique item ids!")
        else:
            ids.append(robot['init_args']['item_id'])
    for item in items:
        if item['init_args']['item_id'] in ids:
            raise ValueError("Robots and Items must have unique item ids!")
        else:
            ids.append(item['init_args']['item_id'])
