import sys
import glob
import errno
import csv
import math
import numpy as np


class DataImporter:

    def import_from_csv(self, path, header=True, separator = '\n', by_file = True):
        entries = list()
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


class DataProcessor:

    # waaayy to go jeff...
    def remove_extra_entry(self, entries):
        for entry in entries:
            for element in entry:
                # pop last element that is blank, this is an inplace operator
                element.pop(len(element)-1)
        return entries

    def fixed_length_sampler(self, entries, fixed_length=250):
        fl_entries = list()
        for entry in entries:
            if len(entry) >= fixed_length:
                fl_entry = list()
                length = float(len(entry))
                for i in range(fixed_length):
                    fl_entry.append(entry[int(math.ceil(i * length / fixed_length))])
                fl_entries.append(fl_entry)
            else:
                raise Exception("Fixed length is less than length of entry. \\"
                                "Try reducing the fixed length of a trajectory/keyframe")
        return fl_entries

    def concatenate_trajectory_observations(self, entries):
        observations = list()
        for entry in entries:
            for observation in entry:
                observations.append(observation)
        return observations

    def convert_numpy(self, sequence, type='f'):
        return np.array(sequence, dtype=type)