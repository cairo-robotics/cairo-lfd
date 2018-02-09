#!/usr/bin/env python2.7
from lfd_processor.processing import DataProcessor
from lfd_processor.interfaces import Demonstration, Observation
from lfd_processor.io import DataImporter
from lfd_modeling.visualization import KMeansModelViewer
from lfd_modeling.modeling import KMeansModel
import os

if __name__  == "__main__":

    """
    An example of generating and visualizing KMeans on the raw trajectories from the toy data. 
    """

    importer = DataImporter()
    processor = DataProcessor()
    directory = os.path.dirname(__file__)
    filename = os.path.join(directory, '../toy_data/constrained_trajectories/*.json')
    trajectories = importer.load_json_files(filename)

    # Convert trajectory data into Demonstrations and Observations
    demonstrations = []
    for datum in trajectories["data"]:
        observations = []
        for entry in datum:
            observations.append(Observation(entry))
        demonstrations.append(Demonstration(observations))

    demo_vectors = []
    for demo in demonstrations:
        observation_vectors = []
        for observation in demo.observations:
            position_data = observation.data["robot"]["position"]
            joint_data = observation.data["robot"]["joints"]
            vector = position_data + joint_data
            observation_vectors.append(vector)
        demo_vectors.append(observation_vectors)

    observations = []
    for vec in demo_vectors:
        for ob in vec:
            observations.append(ob)
    np_observations = processor.to_np_array(observations)
    kmm = KMeansModel(np_observations, n_clusters=5)
    kmm.kmeans_fit()

    viewer = KMeansModelViewer(kmm, np_observations)
    viewer.view_3D_clusters(0, 1, 2)
