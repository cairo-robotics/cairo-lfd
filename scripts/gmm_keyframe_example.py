from keyframe.modeling import GausssianMixtureModel
from keyframe.data_processing import DataImporter, DataProcessor
from keyframe.visualization import SamplePointViewer, GaussianMixtureModelViewer
import os

if __name__ == "__main__":

    importer = DataImporter()
    processor = DataProcessor()
    dir = os.path.dirname(__file__)
    filename = os.path.join(dir, "../toy_data/keyframe_data/keyframes.json")
    keyframe_data = importer.load_keyframe_json(filename)

    for key in sorted(keyframe_data.keys()):

        observations = []
        for observation in keyframe_data[key]["data"]:
            sample = processor.convert_observation_dict_to_list(observation)
            observations.append(sample)
        np_observations = processor.convert_to_numpy_array(observations)

        model = GausssianMixtureModel(np_observations)
        model.gmm_fit()
        simulated_samples = model.generate_samples(200)

        gmm_viewer = GaussianMixtureModelViewer(model, np_observations)
        gmm_viewer.view_2D_gaussians(0, 1)

        sample_viewer = SamplePointViewer(simulated_samples)
        sample_viewer.view_3D_scatter()
