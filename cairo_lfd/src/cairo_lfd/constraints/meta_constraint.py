from collections import Counter

from cairo_lfd.constraints.concept_constraints import HeightConstraint, UprightConstraint, Perimeter2DConstraint, OverUnderConstraint
from cairo_lfd.constraints.heuristics import height_heuristic, orientation_heuristic, perimeter_heuristic, over_under_heuristic


class MetaConstraintBuilder():

    def __init__(self, segment_model, meta_constraint):
        self.segment_model = segment_model
        self.static_parameters
        self.meta_constraint = meta_constraint

    def generate_meta_constraints(self, vectors):
        component_id = self._predict_component(vectors)
        parameters = self.segment_model.get_component_parameter_map(
            component_id)

        constraints = self.meta_constraint_class.generate_constraints()

    def _predict_component(self, vectors):
        # using raw observations of a keyframe, assign the keyframe to the most common component id.
        predictions = self.segment_model.predict(vectors)
        predictions_counter = Counter(predictions)
        return predictions_counter.most_common(1)[0][0]


class HeightMetaConstraint():

    def __init__(self, static_parameters):
        self.static_params = static_parameters
        self.constraint_class = HeightConstraint
        self.height_heuristic = height_heuristic

    def generate_constraints(self, heuristic_parameters):
        discrete_heights = self.height_heuristic(heuristic_parameters)
        constraints = []
        for idx, height in enumerate(discrete_heights):
            constraints.append(HeightConstraint(threshold_distance=height, **static_parameters))
        self.constraints = constraints


class UprightMetaConstraint():

    def __init__(self, heuristic_parameters, static_parameters):
        self.heuristic_params = heuristic_parameters
        self.static_params = static_parameters
        self.constraint_class = HeightConstraint
        self.height_heuristic = height_heuristic

    def generate_constraints(self):
        pass


class Perimeter2DMetaConstraint():

    def __init__(self, heuristic_parameters, static_parameters):
        self.heuristic_params = heuristic_parameters
        self.static_params = static_parameters
        self.constraint_class = Perimeter2DConstraint
        self.perimeter_heuristic = perimeter_heuristic

    def generate_constraints(self):
        pass


class OverUnderMetaConstraint():

    def __init__(self, heuristic_parameters, static_parameters):
        self.heuristic_params = heuristic_parameters
        self.static_params = static_parameters
        self.constraint_class = OverUnderConstraint
        self.over_under_heuristic = over_under_heuristic

    def generate_constraints(self):
        pass
