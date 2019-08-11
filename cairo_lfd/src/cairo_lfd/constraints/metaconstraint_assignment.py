from collections import Counter

import numpy as np

from cairo_lfd.constraints.metaconstraints import HeightMetaconstraint, UprightMetaconstraint, OverUnderMetaconstraint, Perimeter2DMetaconstraint
from cairo_lfd.data.vectorization import boolean_within_SOI, boolean_within_perimeter, xy_radial_distance, vectorize_robot_position, vectorize_robot_orientation


class MetaconstraintAssigner():

    def __init__(self, environment, graph, metaconstraint_builders):
        self.env = environment
        self.graph = graph
        self.builders = metaconstraint_builders

    def assign_metaconstraints(self,):
        for node in self.graph.get_keyframe_sequence():
            metaconstraints = [meta for meta in [
                builder.build_metaconstraint(self.graph.nodes[node], self.env) for builder in self.builders] if meta is not None]
            self.graph.nodes[node]['metaconstraints'] = metaconstraints


class HeightMetaconstraintBuilder():

    def __init__(self, heuristic_model, static_parameters):
        self.heuristic_model = heuristic_model
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node, environment=None):
        metaconstraint = HeightMetaconstraint(self.static_parameters)
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        metaconstraint.parameterize_constraints(heuristic_parameters)
        return metaconstraint

    def _get_heuristic_parameters(self, keyframe_node):
        vectors = np.array([vectorize_robot_position(obs) for obs in keyframe_node['observations']])
        return self.heuristic_model.get_parameters(vectors)


class UprightMetaconstraintBuilder():

    def __init__(self, heuristic_model, static_parameters):
        self.heuristic_model = heuristic_model
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node, environment=None):
        metaconstraint = UprightMetaconstraint(static_parameters)
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        metaconstraint.parameterize_constraints(heuristic_parameters)
        return metaconstraint

    def _get_heuristic_parameters(self, keyframe_node):
        vectors = np.array([vectorize_robot_orientation(obs) for obs in keyframe_node['observations']])
        return self.heuristic_model.get_parameters(vectors)


class OverUnderMetaconstraintBuilder():

    def __init__(self, heuristic_model, static_parameters):
        self.heuristic_model = heuristic_model
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node, environment=None):
        if self._validate_keyframe(keyframe_node):
            metaconstraint = OverUnderMetaconstraint(static_parameters)
            heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
            metaconstraint.parameterize_constraints(heuristic_parameters)
            return metaconstraint
        return None

    def _validate_keyframe(self, keyframe_node):
        above_item_id = self.static_parameters["above_item_id"]
        below_item_id = self.static_parameters["below_item_id"]
        boolean_vectors = [boolean_within_perimeter(obs, above_item_id, below_item_id) for obs in keyframe_node['observations']]
        predictions_counter = Counter(predictions)
        return predictions_counter.most_common(1)[0][0]

    def _get_heuristic_parameters(self, keyframe_node):
        return self.heuristic_model.get_parameters(np.array([xy_radial_distance(obs) for obs in keyframe_node['observations']]))


class Perimeter2DMetaconstraintBuilder():

    def __init__(self, heuristic_model, static_parameters):
        self.heuristic_model = heuristic_model
        self.metaconstraint_class = Perimeter2DMetaconstraint(static_parameters)
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node, environment=None):
        if self._validate_keyframe(keyframe_node):
            heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
            self.metaconstraint.parameterize_constraints(heuristic_parameters)
            return self.metaconstraint
        return None

    def _validate_keyframe(self, keyframe_node):
        perimeter_item_id = static_parameters["perimeter_item_id"]
        traversing_item_id = static_parameters["traversing_item_id"]
        boolean_vectors = [boolean_within_perimeter(obs, perimeter_item_id, traversing_item_id) for obs in keyframe_node['observations']]
        predictions_counter = Counter(predictions)
        return predictions_counter.most_common(1)[0][0]

    def _get_heuristic_parameters(self, keyframe_node, environment):
        curr_item_state = self.environment.get_item_state_by_id(static_parameters["perimeter_item_id"])
        return self.heuristic_model.get_parameters(curr_item_state)
 
