from collections import Counter
import copy

from cairo_lfd.constraints.concept_constraints import PlanarConstraint, OrientationConstraint, Perimeter2DConstraint, OverUnderConstraint


class PlanarAutoconstraint():

    def __init__(self, name, static_parameters):
        self.name = name
        self.static_params = static_parameters
        self.constraint_class = PlanarConstraint
        self.valid_constraints = []

    def parameterize_constraints(self, discrete_heights):
        constraints = []
        # More constrained is higher height so we sort in descending order.
        discrete_heights = sorted(discrete_heights, reverse=True)
        for idx, height in enumerate(discrete_heights):
            constraints.append(PlanarConstraint(constraint_id=(self.name, idx),
                                                threshold_distance=height, **self.static_params))
        self.constraints = constraints


class OrientationAutoconstraint():

    def __init__(self, name, static_parameters):
        self.name = name
        self.static_params = static_parameters
        self.constraint_class = OrientationConstraint

    def parameterize_constraints(self, avg_orientation, angles_of_deviation):
        constraints = []
        angles_of_deviation = sorted(angles_of_deviation)
        for idx, angle in enumerate(angles_of_deviation):
            constraints.append(OrientationConstraint(constraint_id=(self.name, idx),
                                                     reference_orientation=avg_orientation, threshold_angle=angle, **self.static_params))
        self.constraints = constraints


class Perimeter2DAutoconstraint():

    def __init__(self, name, static_parameters):
        self.name = name
        self.static_params = static_parameters
        self.constraint_class = Perimeter2DConstraint

    def parameterize_constraints(self, heuristic_parameters=None):
        # There is nothing to parameterize. The function parameters exist to support interfaceing to AutoConstraint Builders.
        self.constraints = [Perimeter2DConstraint(constraint_id=(self.name, 0), **self.static_params)]


class OverUnderAutoconstraint():

    def __init__(self, name, static_parameters):
        self.name = name
        self.static_params = copy.deepcopy(static_parameters)
        self.constraint_class = OverUnderConstraint

    def parameterize_constraints(self, discrete_radii):
        constraints = []
        discrete_radii = sorted(discrete_radii)
        for idx, threshold_radius in enumerate(discrete_radii):
            if self.static_params.get("below_item_id") is None:
                self.static_params["below_item_id"] = -1
            constraints.append(OverUnderConstraint(constraint_id=(self.name, idx),
                                                   threshold_distance=threshold_radius, **self.static_params))
        self.constraints = constraints
