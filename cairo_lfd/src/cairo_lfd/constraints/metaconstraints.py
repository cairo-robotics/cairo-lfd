from collections import Counter

from cairo_lfd.constraints.concept_constraints import HeightConstraint, UprightConstraint, Perimeter2DConstraint, OverUnderConstraint


class HeightMetaconstraint():

    def __init__(self, static_parameters):
        self.static_params = static_parameters
        self.constraint_class = HeightConstraint
        self.valid_constraints = []

    def parameterize_constraints(self, discrete_heights):
        constraints = []
        for idx, height in enumerate(discrete_heights):
            constraints.append(HeightConstraint(constraint_id=idx,
                                                threshold_distance=height, **self.static_params))
        self.constraints = constraints


class UprightMetaconstraint():

    def __init__(self, static_parameters):
        self.static_params = static_parameters
        self.constraint_class = UprightConstraint

    def generate_constraints(self, heuristic_parameters):
        pass


class Perimeter2DMetaconstraint():

    def __init__(self, static_parameters):
        self.heuristic_params = heuristic_parameters
        self.static_params = static_parameters
        self.constraint_class = Perimeter2DConstraint
        self.perimeter_heuristic = perimeter_heuristic

    def generate_constraints(self, heuristic_parameters=None):
        pass


class OverUnderMetaconstraint():

    def __init__(self, static_parameters):
        self.static_params = static_parameters
        self.constraint_class = OverUnderConstraint
        self.over_under_heuristic = over_under_heuristic

    def generate_constraints(self, discrete_radii):
        constraints = []
        for idx, radii in enumerate(discrete_heights):
            constraints.append(OverUnderConstraint(constraint_id=idx,
                                                threshold_distance=height, **self.static_params))
        self.constraints = constraints
