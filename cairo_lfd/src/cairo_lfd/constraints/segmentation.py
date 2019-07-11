"""
The segmentation.py module supports segmenting demonstration data for use in autonomous constraint assignment.
"""
import numpy as np
from sklearn import mixture


class MetaConstraintSegmentation():

    def __init__(self, segment_model):
        self.segment_model = segment_model

    def predict_component(self, vectors):
        pass


class BayesianGMMSegmentModel():

    def __init__(self, training_data, n_components):
        self.training_data = training_data
        self.n_components = n_components
        self.n_samples = len(demonstrations)
        self._fit_model()

    def _fit_model(self):
        # Build model using every observation available
        X = np.array(training_data)
        if self.n_samples < self.n_components:
            self.model = mixture.BayesianGaussianMixture(n_components=X.shape[0]).fit(X)
        else:
            self.model = mixture.BayesianGaussianMixture(n_components=n_components).fit(X)

    def predict(self, vector):
        # Predict segmentation using trained model
        X = np.array(vector)
        prediction = self.model.predict(X)
        return prediction


class LabelBasedSegmenter():
    def __init__(self):
        pass

    def segment(self, observations):

        # Initialze
        observations.data[0]['segment'] = 1
        segchange = False  # track whether segment increment is necessary

        # Parse
        for i in range(1, len(observations)):  # iterate through blocks
            # iterate through items
            for j in range(len(observations[i].data['items'])):
                # iterate through in_contact
                for key in observations.data[i]['items'][j]['in_contact']:
                    if observations.data[i]['items'][j]['in_contact'][key] != observations.data[i - 1]['items'][j]['in_contact'][key]:
                        segchange = True
            # iterate through in_contact for robot
            for key in observations.data[i]['robot']['in_contact']:
                if observations.data[i]['robot']['in_contact'][key] != observations.data[i - 1]['robot']['in_contact'][key]:
                    segchange = True
            if observations.data[i]['robot']['in_SOI'] != observations.data[i - 1]['robot']['in_SOI']:
                segchange = True
            if segchange is True:
                observations.data[i]['segment'] = observations.data[i -
                                                                    1]['segment'] + 1
                segchange = False
            else:
                observations.data[i]['segment'] = observations.data[i - 1]['segment']