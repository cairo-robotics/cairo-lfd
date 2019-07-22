from collections import Counter

from cairo_lfd.constraints.concept_constraints import HeightConstraint, OrientationConstraint, Perimeter2DConstraint, OverUnderConstraint
from cairo_lfd.heuristics import height_heuristic, orientation_heuristic, perimeter_heuristic, over_under_heuristic


class MetaConstraintAssignment():

    def __init__(self, segment_model):
        self.segment_model = segment_model

    def predict_component(self, vectors):
        # using raw observations of a keyframe, assign the keyframe to the most common component id.
        predictions = self.segment_model.predict(vectors)
        predictions_counter = Counter(predictions)
        return occurence_count.most_common(1)[0][0]


class MetaConstraintFactory():

    def __init__(self, self.segment_model, meta_constraints):
        self.segment_model = segment_model
        self.meta_constraint = meta_constraint

    def generate_meta_constraint(self, component_id):
        parameters = self.segment_model.get_component_parameter_map(
            component_id)
        constraints = self.meta_constraint_class.generate_constraints()


class HeightMetaConstraint():

    def __init__(self, segment_model, heuristic_parameters, static_parameters):
        self.heuristic_params = heuristic_parameters
        self.static_params = static_parameters
        self.constraint_class = HeightConstraint
        self.height_heuristic = height_heuristic

    def generate_constraints(self):
        discrete_heights = self.height_heuristic(self.heuristic_parameters)
        constraints = []
        for idx, height in enumerate(discrete_heights):
            constraints.append(HeightConstraint(**static_parameters, threshold_distance=height))
        self.constraints = constraints


class OrientationMetaConstraint():

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
        self.height_heuristic = height_heuristic

    def generate_constraints(self):
        pass


class HeightMetaConstraint():

    def __init__(self, heuristic_parameters, static_parameters):
        self.heuristic_params=heuristic_parameters
        self.static_params=static_parameters
        self.constraint_class=HeightConstraint
        self.height_heuristic=height_heuristic

    def generate_constraints(self):
        pass
