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

    def get_component_parameters(self, component_id):
        return {
            "mean": self._get_means(component_id),
            "covar": self._get_covariances(component_id)
        }

    def _get_means(self, component_id):
        return self.model.means_[component_id]

    def _get_covariances(self, component_id):
        return self.model.covariances_[component_id]

    def _predict(self, vector):
        # Predict segmentation using trained model
        X = np.array(vector)
        prediction = self.model.predict(X)
        return prediction

    def predict(self, vectors):
        # using raw observations of a keyframe, assign the keyframe to the most common component id.
        predictions = self._predict(vectors)
        predictions_counter = Counter(predictions)
        return predictions_counter.most_common(1)[0][0]
