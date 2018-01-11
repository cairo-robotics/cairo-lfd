import sys
import glob
import errno
import csv
import math
import numpy as np
import pprint


class DataImporter:

    def import_csv_to_list(self, path, header=True):
        entries = []
        files = glob.glob(path)
        for name in files:
            try:
                with open(name, encoding='utf') as f:
                    reader = csv.reader(f)
                    trajectory = list(reader)
                    if header:
                        trajectory.pop(0)
                    entries.append(trajectory)
            except IOError as exc:
                if exc.errno != errno.EISDIR:
                    raise  # Propagate other kinds of IOError.
        return entries

    def import_csv_to_dict(self, path):
        entries = {}
        entries["trajectories"] = []
        files = glob.glob(path)
        for name in files:
            try:
                with open(name, encoding='utf') as f:
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


class DataProcessor:

    def fixed_length_sampler(self, entries, fixed_length=250):
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

    def concatenate_trajectory_observations(self, entries):
        observations = []
        for entry in entries:
            for observation in entry:
                observations.append(observation)
        return observations

    def convert_trajectory_dict_to_list(self, trajectory_dict, key_order=["PoseX", "PoseY", "PoseZ", "OrienX", "OrienY", "OrienZ", "OrienW", "time"]):
        trajectory_list = []
        for key in key_order:
            print(key)
            trajectory_list.append(trajectory_dict.key)
        return trajectory_list

    def convert_to_numpy(self, sequence, type='f'):
        return np.array(sequence, dtype=type)

if __name__ == "__main__":
    importer = DataImporter()
    processor = DataProcessor()
    trajectories_dict = importer.import_csv_to_dict('../toy_data/*.csv')
    trajectories_list = []
    print(len(trajectories_dict["trajectories"]))
    for t in trajectories_dict["trajectories"]:
        for observation in t["observations"]:
            trajectory = []
            processor.convert_trajectory_dict_to_list(t)
