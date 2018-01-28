#!/usr/bin/env python


import networkx as nx
from keyframe.modeling import GaussianMixtureModel
from keyframe.data_processing import DataImporter, DataProcessor

#from keyframe.data_processing import DataImporter, DataProcessor

import os, signal


class TaskGraph(object):

    def __init__(self):
        self.task_graph = nx.DiGraph()
        self._processor = DataProcessor()
        self._tail()


    def add_node(self, samples):
        observations = []
        for observation in samples["data"]:
            sample = self._processor.convert_observation_dict_to_list(observation)
            observations.append(sample)

        np_observations = self._processor.to_np_array(observations)
        model = GaussianMixtureModel(np_observations)
        model.gmm_fit()


    def check_sample_points(self):
        pass

def main():

    task_graph = TaskGraph()

    importer = DataImporter()
    processor = DataProcessor()
    dir = os.path.dirname(__file__)
    filename = os.path.join(dir, "../../../lfd_keyframe_examples/toy_data/keyframe_data/keyframes.json")
    keyframe_data = importer.load_json_files(filename)

    task_graph.add_node(keyframe_data['1'])

    for key in sorted(keyframe_data.keys()):
        pass




    print

if __name__ == '__main__':
    main()
