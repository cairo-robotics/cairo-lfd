from collections import Counter

from cairo_lfd.constraints.concept_constraints import HeightConstraint, UprightConstraint, Perimeter2DConstraint, OverUnderConstraint
from cairo_lfd.constraints.heuristics import height_heuristic, orientation_heuristic, perimeter_heuristic, over_under_heuristic


class HeightMetaconstraint():

    def __init__(self, static_parameters):
        self.static_params = static_parameters
        self.constraint_class = HeightConstraint
        self.height_heuristic = height_heuristic

    def parameterize_constraints(self, heuristic_parameters):
        discrete_heights = self.height_heuristic(**heuristic_parameters)
        constraints = []
        for idx, height in enumerate(discrete_heights):
            constraints.append(HeightConstraint(constraint_id=idx,
                                                threshold_distance=height, **self.static_params))
        self.constraints = constraints


class UprightMetaconstraint():

    def __init__(self, static_parameters):
        self.heuristic_params = heuristic_parameters
        self.static_params = static_parameters
        self.constraint_class = HeightConstraint
        self.height_heuristic = height_heuristic

    def generate_constraints(self, heuristic_parameters):
        pass


class Perimeter2DMetaconstraint():

    def __init__(self, static_parameters):
        self.heuristic_params = heuristic_parameters
        self.static_params = static_parameters
        self.constraint_class = Perimeter2DConstraint
        self.perimeter_heuristic = perimeter_heuristic

    def generate_constraints(self, heuristic_parameters):
        pass


class OverUnderMetaconstraint():

    def __init__(self, static_parameters):
        self.static_params = static_parameters
        self.constraint_class = OverUnderConstraint
        self.over_under_heuristic = over_under_heuristic

    def generate_constraints(self, heuristic_parameters):
        discrete_radii = self.over_under_heuristic(**heuristic_parameters)
        constraints = []
        for idx, height in enumerate(discrete_heights):
            constraints.append(HeightConstraint(constraint_id=idx,
                                                threshold_distance=height, **self.static_params))
        self.constraints = constraints
