"""
The segmentation.py module supports segmenting demonstration data for use in autonomous constraint assignment.
"""


import numpy as np
from sklearn import mixture


class BayesianGMMSegmentModel():

    def __init__(self, training_data, n_components):
        self.training_data = training_data
        self.dimensionality = training_data.shape[1]
        self.n_components = n_components
        self.n_samples = len(training_data)

    def fit(self):
        # Build model using every observation available
        X = self.training_data
        if self.n_samples < self.n_components:
            self.model = mixture.BayesianGaussianMixture(
                n_components=X.shape[0], covariance_type='full', weight_concentration_prior=1e-2,
                weight_concentration_prior_type='dirichlet_process',
                mean_precision_prior=1e-2, covariance_prior=1e0 * np.eye(self.dimensionality),
                init_params="random", max_iter=100, random_state=2).fit(X)
        else:
            self.model = mixture.BayesianGaussianMixture(
                n_components=self.n_components, covariance_type='full', weight_concentration_prior=1e-2,
                weight_concentration_prior_type='dirichlet_process',
                mean_precision_prior=1e-2, covariance_prior=1e0 * np.eye(self.dimensionality),
                init_params="random", max_iter=100, random_state=2).fit(X)

    def get_means(self, component_id):
        return self.model.means_[component_id]

    def get_covariances(self, component_id):
        return self.model.covariances_[component_id]

    def predict(self, vector):
        # Predict segmentation using trained model
        X = np.array(vector)
        prediction = self.model.predict(X)
        return prediction