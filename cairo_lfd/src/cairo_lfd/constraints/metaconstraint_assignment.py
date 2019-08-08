from collections import Counter
from cairo_lfd.constraints.metaconstraints import HeightMetaconstraint, UprightMetaconstraint, OverUnderMetaconstraint, Perimeter2DMetaconstraint
from cairo_lef.constraints.heuristics import get_segmentation_parameters


############################################
# Metaconstraint Assignment and Generation #
############################################

"""
There are lot of injected functions (but not classical DI) that couple these classes to external functions. TODO: Utilize DI and generalize 
"""


class MetaconstraintAssigner():

    def __init__(self, environment, graph, metaconstraint_generators):
        self.env = environment
        self.graph = graph
        self.generators = metaconstraint_generators

    def assign_metaconstraints():
        for node in self.graph.get_keyframe_sequence():
            metaconstraints = [meta for meta in [
                gen.generate_metaconstraint(keyframe_node, self.env) for gen in self.generators] if meta is not None]

            keyframe_node['metaconstraints'] = metaconstraints


class HeightMetaconstraintGenerator():

    def __init__(self, segment_model, static_parameters, vectorizor):
        self.segment_model = segment_model
        self.metaconstraint = HeightMetaconstraint(static_parameters)
        self.vectorizor = vectorizor
        self.static_parameters = static_parameters

    def generate_metaconstraint(self, keyframe_node):
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        self.metaconstraint.parameterize_constraints(heuristic_parameters)
        return metaconstraint

    def _get_heuristic_parameters(self, keyframe_node, environment=None):
        vectors = np.array([self.vectorizor(obs) for obs in keyframe_node['observations']])
        component_id = self.segment_model.predict(vectors)
        return get_segmentation_parameters(self.segment_model, component_id)


class UprightMetaconstraintGenerator():

    def __init__(self, segment_model, static_parameters, vectorizor):
        self.segment_model = segment_model
        self.metaconstraint = UprightMetaconstraint(static_parameters)
        self.vectorizor = vectorizor
        self.static_parameters = static_parameters

    def generate_metaconstraint(self, keyframe_node):
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        metaconstraint.parameterize_constraints(heuristic_parameters)
        return metaconstraint

    def _get_heuristic_parameters(self, keyframe_node, environment=None):
        vectors = np.array([self.vectorizor(obs) for obs in keyframe_node['observations']])
        component_id = self.segment_model.predict(vectors)
        return get_segmentation_parameters(self.segment_model, component_id)


class OverUnderMetaconstraintGenerator():

    def __init__(self, static_parameters, vectorizor, metadata_validator):
        self.metaconstraint_class = OverUnderMetaconstraint(static_parameters)
        self.static_parameters = static_parameters
        self.metadata_validator = metadata_validator

    def generate_metaconstraint(self, keyframe_node, environment):
        metaconstraint = self.meta_constraint_class(self.static_parameters)
        heuristic_constraints = self._get_heuristic_parameters(keyframe_node, environment)
        metaconstraint.parameterize_constraints(heuristic_parameters)
        return metaconstraint

    def _get_heuristic_parameters(self, keyframe_node, environment):

        if self.metadata_validator(keyframe_node, self.static_parameters):

            return self.segment_model.get_heuristic_parameters(component_id)

class Perimeter2DMetaconstraintGenerator():

    def __init__(self, static_parameters, vectorizor, metadata_validator):
        self.metaconstraint_class = Perimeter2DMetaconstraint
        self.static_parameters = static_parameters
        self.metadata_validator = metadata_validator

    def generate_metaconstraint(self, keyframe_node, environment):
        metaconstraint = self.meta_constraint_class(self.static_parameters)
        heuristic_constraints = self._get_heuristic_parameters(keyframe_node, environment)
        metaconstraint.parameterize_constraints(heuristic_parameters)
        return metaconstraint

    def _get_heuristic_parameters(self, keyframe_node, environment):

        if self.metadata_validator(keyframe_node, self.static_parameters):

            return self.segment_model.get_heuristic_parameters(component_id)

#############################
# Metaconstraint Validation #
#############################


def over_under_validator(keyframe_node, static_parameters):
    above_item_id = static_parameters["above_item_id"]
    under_item_id = below_item_id["below_item_id"]


def perimeter_validator(keyframe_node, static_parameters):
    pass



