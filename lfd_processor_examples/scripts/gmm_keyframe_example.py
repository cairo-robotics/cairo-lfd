#!/usr/bin/env python2.7

from lfd_processor.processing import DataProcessor
from lfd_processor.data_io import DataImporter
from lfd_modeling.modeling import GaussianMixtureModel
from lfd_modeling.visualization import SamplePointViewer, GaussianMixtureModelViewer
import os

if __name__ == "__main__":

    """
    An example of generating and visualizing GMM on the generated keyframes.json file from the toy data. 
    """

    importer = DataImporter()
    processor = DataProcessor()
    dir = os.path.dirname(__file__)
    filename = os.path.join(dir, "../toy_data/keyframe_data/keyframes.json")
    keyframe_data = importer.load_json_file(filename)

    for key in sorted(keyframe_data.keys()):

        observations = []
        for observation in keyframe_data[key]["data"]:
            sample = processor.convert_observation_dict_to_list(observation)
            observations.append(sample)
        np_observations = processor.to_np_array(observations)

        model = GaussianMixtureModel(np_observations)
        model.gmm_fit()
        simulated_samples = model.generate_samples(200)

        gmm_viewer = GaussianMixtureModelViewer(model, np_observations)
        gmm_viewer.view_2D_gaussians(0, 1)

        sample_viewer = SamplePointViewer()
        sample_viewer.view_3D_scatter(simulated_samples, 0, 1, 2)
