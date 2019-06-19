#!/usr/bin/python

import os

from lfd.environment import Demonstration, Observation
from lfd.data_io import DataImporter
from lfd.data_conversion import vectorize_demonstration
from lfd.segmentation import DemonstrationSegmentGenerator, VariationalGMMSegmenter
from modeling.graphing import SegmentationGraphGenerator, SegmentationGraph
import networkx as nx
import matplotlib.pyplot as plt


def get_cycle_edges(graph):
    try:
        return [(from_edge, to_edge) for from_edge, to_edge in list(nx.find_cycle(graph, orientation='original'))]
    except nx.exception.NetworkXNoCycle as e:
        return []


def main():
    # Build model
    dir_path = os.path.join(os.path.dirname(
        os.path.realpath(__file__)), 'place_atop_demos')
    training_data = DataImporter().load_json_files(os.path.join(dir_path, '*.json'))

    # Convert imported data into Demonstrations and Observations
    demonstrations = []
    for datum in training_data["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(observations))

    seg_generator = DemonstrationSegmentGenerator
    gmm_segmenter = VariationalGMMSegmenter(
        demonstrations, vectorize_demonstration, n_components=25)
    model = gmm_segmenter.model
    # Deploy model to segment each demonstration
    demo_segmenter = DemonstrationSegmentGenerator(gmm_segmenter)

    # segments = demo_segmenter.segment_demonstrations(demonstrations)
    segmentation_graph_generator = SegmentationGraphGenerator(demo_segmenter)
    adjacency_list = segmentation_graph_generator.build_weighted_adjacency_list(demonstrations)
    filtered_list = []
    seg_graph = SegmentationGraph()
    seg_graph.add_weighted_edges_from(adjacency_list)
    cycle_edges = get_cycle_edges(seg_graph)
    print(cycle_edges)
    while len(cycle_edges) > 0:
        for edge in cycle_edges:
            seg_graph.get_edge_data(*edge[0:2])["weight"] -= 1
            if seg_graph.get_edge_data(*edge[0:2])["weight"] == 0:
                seg_graph.remove_edge(*edge[0:2])
        cycle_edges = get_cycle_edges(seg_graph)

    nx.draw_spring(seg_graph)
    plt.show()

if __name__ == '__main__':
    main()
