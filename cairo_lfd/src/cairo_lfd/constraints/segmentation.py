"""
The segmentation.py module supports segmenting demonstration data for use in autonomous constraint assignment.
"""


import numpy as np
from sklearn import mixture


class BayesianGMMSegmentModel():

    def __init__(self, training_data, n_components):
        self.training_data = training_data
        self.n_components = n_components
        self.n_samples = len(training_data)
        self._fit_model()

    def _fit_model(self):
        # Build model using every observation available
        X = self.training_data
        if self.n_samples < self.n_components:
            self.model = mixture.BayesianGaussianMixture(
                n_components=X.shape[0], max_iter=500).fit(X)
        else:
            self.model = mixture.BayesianGaussianMixture(
                n_components=self.n_components, max_iter=500).fit(X)

    def get_means(self, component_id):
        return self.model.means_[component_id]

    def get_covariances(self, component_id):
        return self.model.covariances_[component_id]

    def predict(self, vector):
        # Predict segmentation using trained model
        X = np.array(vector)
        prediction = self.model.predict(X)
        return prediction