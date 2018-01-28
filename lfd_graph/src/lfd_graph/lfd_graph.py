#!/usr/bin/env python


import networkx as nx
from keyframe.data_processing import DataImporter, DataProcessor
#from keyframe.data_processing import DataImporter, DataProcessor

import os, signal


class TaskGraph(object):

    def __init__(self, GMM, samples, keyframe_type="standard"):
        self.task_graph = nx.DiGraph()


    def add_node(self, samples):
        pass

    def check_sample_points(self):
        pass

def main():
    importer = DataImporter()
    processor = DataProcessor()
    dir = os.path.dirname(__file__)
    filename = os.path.join(dir, "../../lfd_keyframe_examples/toy_data/keyframe_data/keyframes.json")
    keyframe_data = importer.load_json_files(filename)
    



    print

if __name__ == '__main__':
    main()
