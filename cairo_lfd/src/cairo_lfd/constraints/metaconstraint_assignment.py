from collections import Counter
from cairo_lfd.constraints.metaconstraints import HeightMetaconstraint, UprightMetaconstraint, OverUnderMetaconstraint, Perimeter2DMetaconstraint
from cairo_lfd.constraints.heuristics import get_segmentation_parameters
from cairo_lfd.data.vectorization import boolean_within_SOI, boolean_within_perimeter, xy_radial_distance, vectorize_robot_position
from cairo_lfd.constraints.heuristics import height_heuristic, orientation_heuristic, perimeter_heuristic, over_under_heuristic

"""
There are lot of injected functions (but not classical DI) that couple these classes to external functions. TODO: Utilize DI and generalize 
"""


class MetaconstraintAssigner():

    def __init__(self, graph, metaconstraint_builders):
        self.graph = graph
        self.builders = metaconstraint_builders

    def assign_metaconstraints():
        for node in self.graph.get_keyframe_sequence():
            metaconstraints = [meta for meta in [
                builder.build_metaconstraint(keyframe_node) for builder in self.builders] if meta is not None]
            keyframe_node['metaconstraints'] = metaconstraints


class HeightMetaconstraintBuilder():

    def __init__(self, segment_model, static_parameters):
        self.segment_model = segment_model
        self.metaconstraint = HeightMetaconstraint(static_parameters)
        self.heuristic_func = height_heuristic
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node):
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        self.metaconstraint.parameterize_constraints(heuristic_parameters)
        return metaconstraint

    def _get_heuristic_parameters(self, keyframe_node, environment=None):
        vectors = np.array([vectorize_robot_position(obs) for obs in keyframe_node['observations']])
        component_id = self.segment_model.predict(vectors)
        return self.height_heuristic(self.segment_model.get_component_parameters(component_id))


class UprightMetaconstraintBuilder():

    def __init__(self, segment_model, static_parameters, vectorizor):
        self.segment_model = segment_model
        self.metaconstraint = UprightMetaconstraint(static_parameters)
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node):
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        metaconstraint.parameterize_constraints(heuristic_parameters)
        return metaconstraint

    def _get_heuristic_parameters(self, keyframe_node):
        vectors = np.array([self.vectorizor(obs) for obs in keyframe_node['observations']])
        component_id = self.segment_model.predict(vectors)
        return self.orientation_heuristic(self.segment_model.get_component_parameters(component_id))


class OverUnderMetaconstraintBuilder():

    def __init__(self, static_parameters):
        self.metaconstraint = OverUnderMetaconstraint(static_parameters)
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node):
        if self._validate_keyframe(keyframe_node):
            heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
            self.metaconstraint.parameterize_constraints(heuristic_parameters)
            return metaconstraint
        return None

    def _validate_keyframe(self, keyframe_node):
        above_item_id = self.static_parameters["above_item_id"]
        below_item_id = self.static_parameters["below_item_id"]
        boolean_vectors = [boolean_within_perimeter(obs, above_item_id, below_item_id) for obs in keyframe_node['observations']]
        predictions_counter = Counter(predictions)
        return predictions_counter.most_common(1)[0][0]

    def _get_heuristic_parameters(self, keyframe_node):
        return np.array([xy_radial_distance(obs) for obs in keyframe_node['observations']])


class Perimeter2DMetaconstraintBuilder():

    def __init__(self, static_parameters):
        self.metaconstraint_class = Perimeter2DMetaconstraint(static_parameters)
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node):
        if self._validate_keyframe(keyframe_node):
            heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
            self.metaconstraint.parameterize_constraints(heuristic_parameters)
            return metaconstraint
        return None

    def _validate_keyframe(self, keyframe_node):
        perimeter_item_id = static_parameters["perimeter_item_id"]
        traversing_item_id = static_parameters["traversing_item_id"]
        boolean_vectors = [boolean_within_perimeter(obs, perimeter_item_id, traversing_item_id) for obs in keyframe_node['observations']]
        predictions_counter = Counter(predictions)
        return predictions_counter.most_common(1)[0][0]

    def _get_heuristic_parameters(self, keyframe_node):

        def _get_heuristic_parameters(self, keyframe_node):
            return np.array([xy_radial_distance(obs) for obs in keyframe_node['observations']])
