from collections import Counter

from pyquaternion import Quaternion
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
        predictions = [(self.model.predict([vector]), vector) for vector in self.model.training_data]
        for id_ in range(0, self.model.n_components):
            component_vectors = [vector for pred, vector in predictions if pred == id_]
            parameters_by_component[id_] = self.orientation_heuristic(component_vectors)
        self.parameters = parameters_by_component
        self.fitted = True

    def get_parameters(self, vectors):
        if self.fitted:
            component_id = self.assign_to_component(vectors)
            return self.parameters[component_id]
        else:
            raise UnfittedModelError("The heuristic model needs to be fitted to the data in order to retrieve heuristic parameters for a set of vectorized observations.")

    def assign_to_component(self, vectors):
        # using raw observations of a keyframe, assign the keyframe to the most common component id.
        predictions = self.model.predict(vectors)
        predictions_counter = Counter(predictions)
        return predictions_counter.most_common(1)[0][0]

    def orientation_heuristic(self, orientations):
        if len(orientations) != 0:
            avg_q = self._get_average_quaternion(orientations)
            angle_of_deviations = self._get_angle_of_deviations(avg_q, orientations)
            return avg_q, angle_of_deviations
        return None, None

    def _get_average_quaternion(self, orientations):
        # Based on:
        #
        # Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman.
        # "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30,
        # no. 4 (2007): 1193-1197.
        # Link: https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
        #
        # Code based on:
        #
        # Tolga Birdal. "averaging_quaternions" Matlab code.
        # http://jp.mathworks.com/matlabcentral/fileexchange/40098-tolgabirdal-averaging-quaternions
        Q = np.matrix(orientations)
        M = Q.shape[0]
        A = np.zeros(shape=(4, 4))

        for i in range(0, M):
            q = Q[i, :]
            if np.asarray(q)[0][3] < 0:
                q = -1 * q
            A = np.outer(q, q) + A

        # scale
        A = (1.0 / M) * A

        evals, evecs = np.linalg.eig(A)
        evecs = evecs[:, evals.argsort()[::-1]]
        return np.real(evecs[:, 0])

    def _get_angle_of_deviations(self, avg_q, quaternions):
        ref_vec = np.array([1., 0., 0.])  # Unit vector in the +x direction
        average_q = Quaternion(avg_q[3], avg_q[0], avg_q[1], avg_q[2])
        angles = []
        for q in quaternions:
            current_q = Quaternion(q[3], q[0], q[1], q[2])
            average_vec = average_q.rotate(ref_vec)
            current_vec = current_q.rotate(ref_vec)
            angles.append(np.rad2deg(self._angle_between(average_vec, current_vec)))
        std = np.std(angles)
        avg = np.average(angles)
        return np.linspace(avg, avg + 2 * std, 5)

    def _angle_between(self, v1, v2):
        """
        Calculates the angle in radians between vectors.

        Parameters
        ----------
        v1 : array-like
            First vector.
        v2 : array-like
            Second vector.

        Returns
        -------
        : float
            Angle between v1 and v1 in radians.
        """
        v1_u = v1 / np.linalg.norm(v1)
        v2_u = v2 / np.linalg.norm(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


class OverUnderHeuristicModel():

    def __init__(self):
        self.fitted = True

    def fit(self):
        pass

    def get_parameters(self, vectors):
        if self.fitted:
            return self.over_under_heuristic(vectors)
        else:
            raise UnfittedModelError("The heuristic model needs to be fitted to the data in order to retrieve heuristic parameters for a set of vectorized observations.")

    def over_under_heuristic(self, radial_distances):
        std = np.std(radial_distances)
        avg = np.average(radial_distances)
        return np.linspace(avg - std / 2, avg + std / 2, 5)


class PerimeterHeuristicModel():

    def __init__(self):
        self.fitted = True

    def fit(self):
        pass

    def get_parameters(self, ):
        if self.fitted:
            return None
        else:
            raise UnfittedModelError("The heuristic model needs to be fitted to the data in order to retrieve heuristic parameters for a set of vectorized observations.")


