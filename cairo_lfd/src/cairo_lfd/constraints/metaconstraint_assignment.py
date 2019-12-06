from collections import Counter
from functools import partial
from copy import copy
import pudb
import numpy as np

from cairo_lfd.constraints.metaconstraints import HeightMetaconstraint, OrientationMetaconstraint, OverUnderMetaconstraint, Perimeter2DMetaconstraint
from cairo_lfd.data.vectorization import vectorize_demonstration, boolean_within_SOI, boolean_within_perimeter, xy_radial_distance, vectorize_robot_position, vectorize_robot_orientation, xy_radial_distance

from cairo_lfd.constraints.segmentation import BayesianGMMSegmentModel
from cairo_lfd.constraints.heuristics import HeightHeuristicModel, OrientationHeuristicModel, OverUnderHeuristicModel, PerimeterHeuristicModel


class MetaconstraintAssigner():

    def __init__(self, graph, metaconstraint_builders):
        self.graph = graph
        self.builders = metaconstraint_builders

    def assign_metaconstraints(self):
        for node in self.graph.get_keyframe_sequence():
            for builder in self.builders:
                name, metaconstraints = builder.build_metaconstraint(self.graph.nodes[node])
                if metaconstraints is not None:
                    self.graph.nodes[node]['metaconstraints'][name] = metaconstraints


class MetaconstraintBuilderFactory():

    def __init__(self, configs):
        self.configs = configs
        self.builder_configs = {
            "OrientationMetaconstraintBuilder": {
                "vectorizor": lambda demos: np.vstack(np.array(map(partial(vectorize_demonstration, vectorizors=[vectorize_robot_orientation]), demos))),
                "segmentation_model": BayesianGMMSegmentModel,
                "heuristic_model": OrientationHeuristicModel,
                "metaconstraint_builder": OrientationMetaconstraintBuilder
            },
            "HeightMetaconstraintBuilder": {
                "vectorizor": lambda demos: np.vstack(np.array(map(partial(vectorize_demonstration, vectorizors=[vectorize_robot_position]), demos))),
                "segmentation_model": BayesianGMMSegmentModel,
                "heuristic_model": HeightHeuristicModel,
                "metaconstraint_builder": HeightMetaconstraintBuilder
            },
            "OverUnderMetaconstraintBuilder": {
                "vectorizor": None,
                "segmentation_model": None,
                "heuristic_model": OverUnderHeuristicModel,
                "metaconstraint_builder": OverUnderMetaconstraintBuilder
            },
            "Perimeter2DMetaconstraintBuilder": {
                "vectorizor": None,
                "segmentation_model": None,
                "heuristic_model": PerimeterHeuristicModel,
                "metaconstraint_builder": Perimeter2DMetaconstraintBuilder
            }
        }

    def generate_metaconstraint_builders(self, demonstrations):
        builders = []
        for builder in self.configs["metaconstraint_builders"]:
            builder_config = self.builder_configs[builder["class"]]
            static_params = builder['static_parameters']
            if builder_config["segmentation_model"] is not None and builder_config["vectorizor"] is not None:
                vectorizor = builder_config["vectorizor"]
                X = vectorizor(demonstrations)
                segmentation_model = builder_config["segmentation_model"](X, **builder['segmentation']['init_args'])
                heuristic_model = builder_config["heuristic_model"](segmentation_model)
            else:
                heuristic_model = builder_config["heuristic_model"]()
            heuristic_model.fit()
            builders.append(builder_config["metaconstraint_builder"](builder['name'], heuristic_model, static_params))
        return builders


class HeightMetaconstraintBuilder():

    def __init__(self, name, heuristic_model, static_parameters):
        self.name = name
        self.heuristic_model = heuristic_model
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node):
        metaconstraint = HeightMetaconstraint(self.name, self.static_parameters)
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        metaconstraint.parameterize_constraints(heuristic_parameters)
        return self.name, metaconstraint

    def _get_heuristic_parameters(self, keyframe_node):
        vectors = np.array([vectorize_robot_position(obs) for obs in keyframe_node['observations']])
        return self.heuristic_model.get_parameters(vectors)


class OrientationMetaconstraintBuilder():

    def __init__(self, name, heuristic_model, static_parameters):
        self.name = name
        self.heuristic_model = heuristic_model
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node):
        metaconstraint = OrientationMetaconstraint(self.name, self.static_parameters)
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        metaconstraint.parameterize_constraints(*heuristic_parameters)
        return self.name, metaconstraint

    def _get_heuristic_parameters(self, keyframe_node):
        vectors = np.array([vectorize_robot_orientation(obs) for obs in keyframe_node['observations']])
        return self.heuristic_model.get_parameters(vectors)


class OverUnderMetaconstraintBuilder():

    def __init__(self, name, heuristic_model, static_parameters):
        self.name = name
        self.heuristic_model = heuristic_model
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node):
        if self._validate_keyframe(keyframe_node):
            metaconstraint = OverUnderMetaconstraint(self.name, self.static_parameters)
            heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
            metaconstraint.parameterize_constraints(heuristic_parameters)
            return self.name, metaconstraint
        return self.name, None

    def _validate_keyframe(self, keyframe_node):
        above_item_id = self.static_parameters["above_item_id"]
        below_item_id = self.static_parameters["below_item_id"]
        boolean_vectors = [boolean_within_SOI(obs, above_item_id, below_item_id) for obs in keyframe_node['observations']]
        counter = Counter(boolean_vectors)
        return counter.most_common(1)[0][0]

    def _get_heuristic_parameters(self, keyframe_node):
        return self.heuristic_model.get_parameters(np.array([xy_radial_distance(obs, self.static_parameters["above_item_id"], self.static_parameters["below_item_id"]) for obs in keyframe_node['observations']]))


class Perimeter2DMetaconstraintBuilder():

    def __init__(self, name, heuristic_model, static_parameters):
        self.name = name
        self.heuristic_model = heuristic_model
        self.static_parameters = static_parameters

    def build_metaconstraint(self, keyframe_node):
        metaconstraint = Perimeter2DMetaconstraint(self.name, self.static_parameters)
        if self._validate_keyframe(keyframe_node):
            heuristic_parameters = self._get_heuristic_parameters()
            metaconstraint.parameterize_constraints(heuristic_parameters)
            return self.name, metaconstraint
        return self.name, None

    def _validate_keyframe(self, keyframe_node):
        perimeter_item_id = self.static_parameters["perimeter_item_id"]
        traversing_item_id = self.static_parameters["traversing_item_id"]
        boolean_vectors = [boolean_within_perimeter(obs, perimeter_item_id, traversing_item_id) for obs in keyframe_node['observations']]
        counter = Counter(boolean_vectors)
        return counter.most_common(1)[0][0]

    def _get_heuristic_parameters(self,):
        return self.heuristic_model.get_parameters()