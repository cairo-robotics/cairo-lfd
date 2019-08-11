from collections import Counter

import numpy as np
from scipy import linalg


class UnfittedModelError(Exception):
    pass


class HeightHeuristicModel():

    def __init__(self, segmentation_model):
        self.model = segmentation_model
        self.fitted = False

    def fit(self):
        parameters_by_component = {}
        for id_ in range(0, self.model.n_components):
            parameters_by_component[id_] = self.height_heuristic(self.model.get_means(id_), self.model.get_covariances(id_))
        self.parameters = parameters_by_component
        self.fitted = True

    def get_parameters(self, vectors):
        if self.fitted:
            component = self.assign_to_component(vectors)
            print component
            return self.parameters[component]
        else:
            raise UnfittedModelError("The heuristic model needs to be fitted to the data in order to retrieve heuristic parameters from by segmentation component.")

    def assign_to_component(self, vectors):
        # using raw observations of a keyframe, assign the keyframe to the most common component id.
        predictions = self.model.predict(vectors)
        predictions_counter = Counter(predictions)
        return predictions_counter.most_common(1)[0][0]

    def height_heuristic(self, mean, covar):
        '''
        Purpose: Finds upper and lower bound of gaussian distribution for a given segment
        and then chooses and returns 5 linearly spaced points within that boundary to use 
        as a heuristic for the height constraint
        '''

        # find the rotation matrix and radii of the axes
        U, s, rotation = linalg.svd(covar)
        radii = np.sqrt(s)

        # now carry on with EOL's answer
        u = np.linspace(0.0, 2.0 * np.pi, 100)
        v = np.linspace(0.0, np.pi, 100)
        x = radii[0] * np.outer(np.cos(u), np.sin(v))
        y = radii[1] * np.outer(np.sin(u), np.sin(v))
        z = radii[2] * np.outer(np.ones_like(u), np.cos(v))

        for i in range(len(x)):
            for j in range(len(x)):
                [x[i, j], y[i, j], z[i, j]] = np.dot(
                    [x[i, j], y[i, j], z[i, j]], rotation) + mean

        # get index of upper and lower bound
        maxz = -100
        maxzindouter = 0
        maxzindinner = 0

        minz = 100
        minzindouter = 0
        minzindinner = 0

        for i in range(len(z)):
            temp = z[i].tolist()
            if max(temp) > maxz:
                maxz = max(temp)
                maxzindouter = i
                maxzindinner = temp.index(max(temp))
            if min(temp) < minz:
                minz = min(temp)
                minzindouter = i
                minzindinner = temp.index(min(temp))

        # use bounds to find 5 acceptable heights
        heights = np.linspace(minz, maxz, 5)
        # print(heights)

        return heights


class OrientationHeuristicModel():

    def __init__(self, segmentation_model):
        self.model = segmentation_model
        self.fitted = False

    def fit(self):
        parameters_by_component = {}
        # we run the predictions on all the data first so that we don't repeatedly predict in the for loop.
        predictions = [(self.model.predict(vector), vector) for vector in self.model.training_data]
        for id_ in range(0, segmentation_mode.n_components):
            component_vectors = [vector for pred, vector in predictions if pred == id_]
            parameters_by_component[id_] = self.orientation_heuristic(component_vectors)
        self.parameters = parameters_by_component
        self.fitted = True

    def get_parameters(self, vectors):
        if self.fitted:
            component = self.assign_to_component(vectors)
            return self.parameters[component_id]
        else:
            raise UnfittedHeuristicModel("The heuristic model needs to be fitted to the data in order to retrieve heuristic parameters for a set of vectorized observations.")

    def assign_to_component(self, vectors):
        # using raw observations of a keyframe, assign the keyframe to the most common component id.
        predictions = self.model.predict(vectors)
        predictions_counter = Counter(predictions)
        return predictions_counter.most_common(1)[0][0]

    def orientation_heuristic(self, orientations):
        pass


class OverUnderHeuristicModel():

    def __init__(self):
        self.fitted = True

    def fit(self):
        pass

    def get_parameters(self, vectors):
        if self.fitted:
            return self.over_under_heuristic(vectors)
        else:
            raise UnfittedHeuristicModel("The heuristic model needs to be fitted to the data in order to retrieve heuristic parameters for a set of vectorized observations.")

    def over_under_heuristic(self, radial_distances):
        pass


class PerimeterHeuristicModel():

    def __init__(self):
        self.fitted = True

    def fit(self):
        pass

    def get_parameters(self, current_item_state):
        if self.fitted:
            return self.perimeter_heuristic(current_item_state)
        else:
            raise UnfittedHeuristicModel("The heuristic model needs to be fitted to the data in order to retrieve heuristic parameters for a set of vectorized observations.")

    def perimeter_heuristic(self, current_item_state):
        current_item_state
