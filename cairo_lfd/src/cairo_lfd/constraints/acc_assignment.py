from collections import Counter
from functools import partial
from copy import copy
import pudb
import numpy as np
import rospy

from cairo_lfd.constraints.autoconstraints import PlanarAutoconstraint, OrientationAutoconstraint, OverUnderAutoconstraint, Perimeter2DAutoconstraint
from cairo_lfd.data.vectorization import vectorize_demonstration, boolean_within_SOI, boolean_within_proximity, boolean_within_perimeter, xy_radial_distance, xy_radial_distance_with_static, vectorize_robot_position, vectorize_robot_orientation

from cairo_lfd.constraints.segmentation import BayesianGMMSegmentModel
from cairo_lfd.constraints.heuristics import HeightHeuristicModel, OrientationHeuristicModel, OverUnderHeuristicModel, PerimeterHeuristicModel


class AutoconstraintAssigner():

    def __init__(self, graph, autoconstraint_builders):
        self.graph = graph
        self.builders = autoconstraint_builders

    def assign_autoconstraints(self):
        for node in self.graph.get_keyframe_sequence():
            for builder in self.builders:
                name, autoconstraints = builder.build_autoconstraint(
                    self.graph.nodes[node])
                if autoconstraints is not None:
                    self.graph.nodes[node]['autoconstraints'][name] = autoconstraints


class AutoconstraintFactory():

    def __init__(self, configs):
        self.configs = configs
        self.builder_configs = {
            "OrientationAutoconstraintBuilder": {
                "vectorizor": lambda demos: np.vstack(np.array(map(partial(vectorize_demonstration, vectorizors=[vectorize_robot_orientation]), demos))),
                "segmentation_model": BayesianGMMSegmentModel,
                "heuristic_model": OrientationHeuristicModel,
                "autoconstraint_builder": OrientationAutoconstraintBuilder
            },
            "PlanarAutoconstraintBuilder": {
                "vectorizor": lambda demos: np.vstack(np.array(map(partial(vectorize_demonstration, vectorizors=[vectorize_robot_position]), demos))),
                "segmentation_model": BayesianGMMSegmentModel,
                "heuristic_model": HeightHeuristicModel,
                "autoconstraint_builder": PlanarAutoconstraintBuilder
            },
            "OverUnderAutoconstraintBuilder": {
                "vectorizor": None,
                "segmentation_model": None,
                "heuristic_model": OverUnderHeuristicModel,
                "autoconstraint_builder": OverUnderAutoconstraintBuilder
            },
            "Perimeter2DAutoconstraintBuilder": {
                "vectorizor": None,
                "segmentation_model": None,
                "heuristic_model": PerimeterHeuristicModel,
                "autoconstraint_builder": Perimeter2DAutoconstraintBuilder
            }
        }

    def generate_autoconstraint_builders(self, demonstrations):
        builders = []
        for builder in self.configs["autoconstraint_builders"]:
            builder_config = self.builder_configs[builder["class"]]
            validation_params = builder.get("validation_parameters", {})
            static_params = builder['static_parameters']
            if builder_config["segmentation_model"] is not None and builder_config["vectorizor"] is not None:
                vectorizor = builder_config["vectorizor"]
                X = vectorizor(demonstrations)
                segmentation_model = builder_config["segmentation_model"](
                    X, **builder['segmentation']['init_args'])
                heuristic_model = builder_config["heuristic_model"](
                    segmentation_model)
            else:
                heuristic_model = builder_config["heuristic_model"]()
            heuristic_model.fit()
            builders.append(builder_config["autoconstraint_builder"](
                builder['name'], heuristic_model, validation_params, static_params))
        return builders


class PlanarAutoconstraintBuilder():

    def __init__(self, name, heuristic_model, validation_params, static_parameters):
        self.name = name
        self.heuristic_model = heuristic_model
        self.validation_params = validation_params
        self.static_parameters = static_parameters

    def build_autoconstraint(self, keyframe_node):
        self._assign_constraint_transitions(keyframe_node)
        autoconstraint = PlanarAutoconstraint(
            self.name, self.static_parameters)
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        autoconstraint.parameterize_constraints(heuristic_parameters)
        return self.name, autoconstraint

    def _get_heuristic_parameters(self, keyframe_node):
        vectors = np.array([vectorize_robot_position(obs)
                            for obs in keyframe_node['observations']])
        return self.heuristic_model.get_parameters(vectors)

    def _assign_constraint_transitions(self, keyframe_node):
        vectors = np.array([vectorize_robot_position(obs)
                            for obs in keyframe_node['observations']])
        component = self.heuristic_model.assign_to_component(vectors)
        keyframe_node["autoconstraint_transitions"].append(self.name + "_" + str(component))


class OrientationAutoconstraintBuilder():

    def __init__(self, name, heuristic_model, validation_params, static_parameters):
        self.name = name
        self.heuristic_model = heuristic_model
        self.validation_params = validation_params
        self.static_parameters = static_parameters

    def build_autoconstraint(self, keyframe_node):
        self._assign_constraint_transitions(keyframe_node)
        autoconstraint = OrientationAutoconstraint(
            self.name, self.static_parameters)
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        autoconstraint.parameterize_constraints(*heuristic_parameters)
        return self.name, autoconstraint

    def _get_heuristic_parameters(self, keyframe_node):
        vectors = np.array([vectorize_robot_orientation(obs)
                            for obs in keyframe_node['observations']])
        return self.heuristic_model.get_parameters(vectors)

    def _assign_constraint_transitions(self, keyframe_node):
        vectors = np.array([vectorize_robot_orientation(obs)
                            for obs in keyframe_node['observations']])
        component = self.heuristic_model.assign_to_component(vectors)
        keyframe_node["autoconstraint_transitions"].append(self.name + "_" + str(component))


class OverUnderAutoconstraintBuilder():

    def __init__(self, name, heuristic_model, validation_params, static_parameters):
        self.name = name
        self.heuristic_model = heuristic_model
        self.validation_params = validation_params
        self.static_parameters = static_parameters

    def build_autoconstraint(self, keyframe_node):
        self._assign_constraint_transitions(keyframe_node)
        if self._validate_keyframe(keyframe_node):
            autoconstraint = OverUnderAutoconstraint(
                self.name, self.static_parameters)
            heuristic_parameters = self._get_heuristic_parameters(
                keyframe_node)
            autoconstraint.parameterize_constraints(heuristic_parameters)
            return self.name, autoconstraint
        return self.name, None

    def _validate_keyframe(self, keyframe_node):
        if self.static_parameters.get("below_item_id", None) is not None:
            above_item_id = self.static_parameters["above_item_id"]
            below_item_id = self.static_parameters["below_item_id"]
            boolean_vectors = [boolean_within_SOI(
                obs, above_item_id, below_item_id) for obs in keyframe_node['observations']]
            counter = Counter(boolean_vectors)
            return counter.most_common(1)[0][0]
        else:
            if self.static_parameters.get("reference_pose", None) is None:
                raise Exception("If using Over Under Autoconstraint without a below object, you must provide a reference_pose in the static parameters!")
            above_item_id = self.static_parameters["above_item_id"]
            boolean_vectors = [boolean_within_proximity(obs, above_item_id, self.static_parameters["reference_pose"]["position"], threshold=self.validation_params.get(
                "threshold", .1)) for obs in keyframe_node['observations']]
            counter = Counter(boolean_vectors)
            return counter.most_common(1)[0][0]

    def _get_heuristic_parameters(self, keyframe_node):
        if self.static_parameters.get("below_item_id", None) is not None: 
            return self.heuristic_model.get_parameters(np.array([xy_radial_distance(obs, self.static_parameters["above_item_id"], self.static_parameters["below_item_id"]) for obs in keyframe_node['observations']]))
        else:
            xy_static = self.static_parameters["reference_pose"]["position"][0:1]
            return self.heuristic_model.get_parameters(np.array([xy_radial_distance_with_static(obs, self.static_parameters["above_item_id"], xy_static) for obs in keyframe_node['observations']]))

    def _assign_constraint_transitions(self, keyframe_node):
        validity = self._validate_keyframe(keyframe_node)
        keyframe_node["autoconstraint_transitions"].append(self.name + "_" + str(validity))


class Perimeter2DAutoconstraintBuilder():

    def __init__(self, name, heuristic_model, validation_params, static_parameters):
        self.name = name
        self.heuristic_model = heuristic_model
        self.validation_params = validation_params
        self.static_parameters = static_parameters

    def build_autoconstraint(self, keyframe_node):
        heuristic_parameters = Perimeter2DAutoconstraint(
            self.name, self.static_parameters)
        self._assign_constraint_transitions(keyframe_node)
        if self._validate_keyframe(keyframe_node):
            heuristic_parameters = self._get_heuristic_parameters()
            heuristic_parameters.parameterize_constraints(heuristic_parameters)
            return self.name, heuristic_parameters
        return self.name, None

    def _validate_keyframe(self, keyframe_node):
        perimeter_item_id = self.static_parameters["perimeter_item_id"]
        traversing_item_id = self.static_parameters["traversing_item_id"]
        boolean_vectors = [boolean_within_perimeter(
            obs, perimeter_item_id, traversing_item_id) for obs in keyframe_node['observations']]
        counter = Counter(boolean_vectors)
        return counter.most_common(1)[0][0]

    def _get_heuristic_parameters(self,):
        return self.heuristic_model.get_parameters()

    def _assign_constraint_transitions(self, keyframe_node):
        validity = self._validate_keyframe(keyframe_node)
        keyframe_node["autoconstraint_transitions"].append(self.name + "_" + str(validity))


