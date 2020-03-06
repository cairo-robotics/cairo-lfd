"""Summary
"""
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


def assign_autoconstraints(self, graph, autoconstraint_builders):
    """
    Assigns Autoconstraints generated from each of the AutoconstraintBuilders to each node in a keyframe graph.

    Parameters
    ----------
    graph : cairo_lfd.graphing.KeyframeGraph
        The Keyframe Graph to which Autoconstraints will be added to.
    autoconstraint_builders : list
        A list of AutonconstraintBuilder objects that each build a parameterized set of Autoconstraints.
    """
    for node in self.graph.get_keyframe_sequence():
        for builder in self.builders:
            name, autoconstraints = builder.build_autoconstraint(
                self.graph.nodes[node])
            if autoconstraints is not None:
                self.graph.nodes[node]['autoconstraints'][name] = autoconstraints


class AutoconstraintFactory():
    """
    A factory-patterned class that creates AutoconstraintBuilder Objects.

    Attributes
    ----------
    builder_configs : dict
        Static set of available AutoconstraintBuilders and their required dependencies.
    configs : dict
        Autoconstraint according to the config.json file imported for a study.
    """
    def __init__(self, configs):
        """
        Parameters
        ----------
        configs : dict
            Autoconstraint according to the config.json file imported for a study.
        """
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
        """
        Generates AutoconstraintBuilders that will ultimately build and learn Autoconstraints on a set of demonstrations.

        Parameters
        ----------
        demonstrations : list
            List of cairo_lfd.environment.Demonstration objects. 

        Returns
        -------
        : list
            List of AutoconstraintBuilders
        """
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
                segmentation_model.fit()
                heuristic_model = builder_config["heuristic_model"](
                    segmentation_model)
            else:
                heuristic_model = builder_config["heuristic_model"]()
            heuristic_model.fit()
            builders.append(builder_config["autoconstraint_builder"](
                builder['name'], heuristic_model, validation_params, static_params))
        return builders


class PlanarAutoconstraintBuilder():
    """
    Builds PlanarAutoconstraints given the provided parameters and heuristic model.

    Attributes
    ----------
    heuristic_model : cairo_lfd.constraints.heuristics.HeightHeuristicModel
        A heuristic model that guides how to parameterize the PlanarAutoconstraints.
    name : str
        Name to apply to the Autoconstraint
    static_parameters : dict
        Static parameters that are predetermined by the user in a config.json file.
    validation_params : dict
        Parameters used to determine if an Autoconstraint is applicable (may not be applicable for a autoconstraint).
    """
    def __init__(self, name, heuristic_model, validation_params, static_parameters):
        """
        Parameters
        ----------
        name : str
            Name to apply to the Autoconstraint
        heuristic_model : cairo_lfd.constraints.heuristics.HeightHeuristicModel
            A heuristic model that guides how to parameterize the PlanarAutoconstraints.
        validation_params : dict
            Parameters used to determine if an Autoconstraint is applicable (may not be applicable for a autoconstraint).
        static_parameters : dict
            Static parameters that are predetermined by the user in a config.json file.
        """
        self.name = name
        self.heuristic_model = heuristic_model
        self.validation_params = validation_params
        self.static_parameters = static_parameters

    def build_autoconstraint(self, keyframe_node):
        """
        Builds the PlanarAutoconstraint object.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)

        Returns
        -------
        self.name : str
            Name of the Autoconstraint
        autoconstraint : cairo_lfd.autoconstraints.PlanarAutoconstraint
            The parameterized Autoconstraint.
        """
        self._assign_constraint_transitions(keyframe_node)
        autoconstraint = PlanarAutoconstraint(
            self.name, self.static_parameters)
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        autoconstraint.parameterize_constraints(heuristic_parameters)
        return self.name, autoconstraint

    def _get_heuristic_parameters(self, keyframe_node):
        """
        Gets the constraint parameters from the learned heuristic model dependent on the passed keyframe_node's observation data.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)

        Returns
        -------
        : list/dict
            The generated parameters from the heuristic model.
        """
        vectors = np.array([vectorize_robot_position(obs)
                            for obs in keyframe_node['observations']])
        return self.heuristic_model.get_parameters(vectors)

    def _assign_constraint_transitions(self, keyframe_node):
        """
        Assigns a unique auto constraint transition string to the keyframe_nodes autoconstraint_transitions field. This is used to determined the constraint transitions akin to the CC-LfD algorithm that generates constraint transition keyframe nodes that cannot be culled.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)

        """
        vectors = np.array([vectorize_robot_position(obs)
                            for obs in keyframe_node['observations']])
        component = self.heuristic_model.assign_to_component(vectors)
        keyframe_node["autoconstraint_transitions"].append(self.name + "_" + str(component))


class OrientationAutoconstraintBuilder():
    """
    Builds OrientationAutoconstraints given the provided parameters and heuristic model.

    Attributes
    ----------
    heuristic_model : cairo_lfd.constraints.heuristics.OrientationHeuristicModel
        A heuristic model that guides how to parameterize the OrientationAutoconstraint.
    name : str
        Name to apply to the Autoconstraint
    static_parameters : dict
        Static parameters that are predetermined by the user in a config.json file.
    validation_params : TYPE
        Parameters used to determine if an Autoconstraint is applicable (may not be applicable for a autoconstraint).
    """
    def __init__(self, name, heuristic_model, validation_params, static_parameters):
        """
        Parameters
        ----------
        name : str
            Name to apply to the Autoconstraint
        heuristic_model : cairo_lfd.constraints.heuristics.OrientationHeuristicModel
            A heuristic model that guides how to parameterize the OrientationAutoconstraint.
        validation_params : dict
            Parameters used to determine if an Autoconstraint is applicable (may not be applicable for a autoconstraint).
        static_parameters : dict
            Static parameters that are predetermined by the user in a config.json file.
        """
        self.name = name
        self.heuristic_model = heuristic_model
        self.validation_params = validation_params
        self.static_parameters = static_parameters

    def build_autoconstraint(self, keyframe_node):
        """
        Builds an OrientationAutoconstraint object.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)

        Returns
        -------
        self.name : str
            Name of the Autoconstraint
        autoconstraint : cairo_lfd.autoconstraints.OrientationAutoconstraint
            The parameterized Autoconstraint.
        """
        self._assign_constraint_transitions(keyframe_node)
        autoconstraint = OrientationAutoconstraint(
            self.name, self.static_parameters)
        heuristic_parameters = self._get_heuristic_parameters(keyframe_node)
        autoconstraint.parameterize_constraints(*heuristic_parameters)
        return self.name, autoconstraint

    def _get_heuristic_parameters(self, keyframe_node):
        """
        Gets the constraint parameters from the learned heuristic model dependent on the passed keyframe_node's observation data.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)

        Returns
        -------
        : list/dict
            The generated parameters from the heuristic model.
        """
        vectors = np.array([vectorize_robot_orientation(obs)
                            for obs in keyframe_node['observations']])
        return self.heuristic_model.get_parameters(vectors)

    def _assign_constraint_transitions(self, keyframe_node):
        """
        Assigns a unique auto constraint transition string to the keyframe_nodes autoconstraint_transitions field. This is used to determined the constraint transitions akin to the CC-LfD algorithm that generates constraint transition keyframe nodes that cannot be culled.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)
        """
        vectors = np.array([vectorize_robot_orientation(obs)
                            for obs in keyframe_node['observations']])
        component = self.heuristic_model.assign_to_component(vectors)
        keyframe_node["autoconstraint_transitions"].append(self.name + "_" + str(component))


class OverUnderAutoconstraintBuilder():
    """
    Builds OverUnderAutoconstraints given the provided parameters and heuristic model.

    Attributes
    ----------
    heuristic_model : cairo_lfd.constraints.heuristics.OverUnderHeuristicModel
        A heuristic model that guides how to parameterize the OverUnderAutoconstraint.
    name : str
        Name to apply to the Autoconstraint
    static_parameters : dict
        Static parameters that are predetermined by the user in a config.json file.
    validation_params : TYPE
        Parameters used to determine if an Autoconstraint is applicable (may not be applicable for a autoconstraint).
    """
    def __init__(self, name, heuristic_model, validation_params, static_parameters):
        """
        Parameters
        ----------
        name : str
            Name to apply to the Autoconstraint
        heuristic_model : cairo_lfd.constraints.heuristics.OverUnderHeuristicModel
            A heuristic model that guides how to parameterize the OverUnderAutoconstraint.
        validation_params : dict
            Parameters used to determine if an Autoconstraint is applicable (may not be applicable for a autoconstraint).
        static_parameters : dict
            Static parameters that are predetermined by the user in a config.json file.
        """
        self.name = name
        self.heuristic_model = heuristic_model
        self.validation_params = validation_params
        self.static_parameters = static_parameters

    def build_autoconstraint(self, keyframe_node):
        """
        Builds an OverUnderAutoconstraint object.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)

        Returns
        -------
        self.name : str
            Name of the Autoconstraint
        autoconstraint : cairo_lfd.autoconstraints.OverUnderAutoconstraint
            The parameterized Autoconstraint.
        """
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
        """
        Validate whether or not a keyframe_node is a viable candidate for the OverUnderAutoconstraint.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)

        Returns
        -------
        : bool
            Whether or not the keyframe node is a candidate for OverUnderAutoconstraint application.

        Raises
        ------
        Exception
            There must be a reference_pose if the static parameters do not contain the below item id.
        """
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
        """
        Gets the constraint parameters from the learned heuristic model dependent on the passed keyframe_node's observation data.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)

        Returns
        -------
        : list/dict
            The generated parameters from the heuristic model.
        """
        if self.static_parameters.get("below_item_id", None) is not None: 
            return self.heuristic_model.get_parameters(np.array([xy_radial_distance(obs, self.static_parameters["above_item_id"], self.static_parameters["below_item_id"]) for obs in keyframe_node['observations']]))
        else:
            xy_static = self.static_parameters["reference_pose"]["position"][0:1]
            return self.heuristic_model.get_parameters(np.array([xy_radial_distance_with_static(obs, self.static_parameters["above_item_id"], xy_static) for obs in keyframe_node['observations']]))

    def _assign_constraint_transitions(self, keyframe_node):
        """
        Assigns a unique auto constraint transition string to the keyframe_nodes autoconstraint_transitions field. This is used to determined the constraint transitions akin to the CC-LfD algorithm that generates constraint transition keyframe nodes that cannot be culled.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)
        """
        validity = self._validate_keyframe(keyframe_node)
        keyframe_node["autoconstraint_transitions"].append(self.name + "_" + str(validity))


class Perimeter2DAutoconstraintBuilder():
    """
    Builds Perimeter2DAutoconstraint given the provided parameters and heuristic model.

    Attributes
    ----------
    heuristic_model : cairo_lfd.constraints.heuristics.PerimeterHeuristicModel
        A heuristic model that guides how to parameterize the Perimeter2DAutoconstraint.
    name : str
        Name to apply to the Autoconstraint
    static_parameters : dict
        Static parameters that are predetermined by the user in a config.json file.
    validation_params : TYPE
        Parameters used to determine if an Autoconstraint is applicable (may not be applicable for a autoconstraint).
    """
    def __init__(self, name, heuristic_model, validation_params, static_parameters):
        """
        Parameters
        ----------
        name : str
            Name to apply to the Autoconstraint
        heuristic_model : cairo_lfd.constraints.heuristics.PerimeterHeuristicModel
            A heuristic model that guides how to parameterize the Perimeter2DAutoconstraint.
        validation_params : dict
            Parameters used to determine if an Autoconstraint is applicable (may not be applicable for a autoconstraint).
        static_parameters : dict
            Static parameters that are predetermined by the user in a config.json file.
        """
        self.name = name
        self.heuristic_model = heuristic_model
        self.validation_params = validation_params
        self.static_parameters = static_parameters

    def build_autoconstraint(self, keyframe_node):
        """
        Builds an Perimeter2DAutoconstraint object.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)

        Returns
        -------
        self.name : str
            Name of the Autoconstraint
        autoconstraint : cairo_lfd.autoconstraints.Perimeter2DAutoconstraint
            The parameterized Autoconstraint.
        """
        heuristic_parameters = Perimeter2DAutoconstraint(
            self.name, self.static_parameters)
        self._assign_constraint_transitions(keyframe_node)
        if self._validate_keyframe(keyframe_node):
            heuristic_parameters = self._get_heuristic_parameters()
            heuristic_parameters.parameterize_constraints(heuristic_parameters)
            return self.name, heuristic_parameters
        return self.name, None

    def _validate_keyframe(self, keyframe_node):
        """
        The parameters are determined exclusively by the data, no node specificity is needed.

        Returns
        -------
        : list/dict
            The generated parameters from the heuristic model.
        """
        perimeter_item_id = self.static_parameters["perimeter_item_id"]
        traversing_item_id = self.static_parameters["traversing_item_id"]
        boolean_vectors = [boolean_within_perimeter(
            obs, perimeter_item_id, traversing_item_id) for obs in keyframe_node['observations']]
        counter = Counter(boolean_vectors)
        return counter.most_common(1)[0][0]

    def _get_heuristic_parameters(self,):
        """
        The parameters are determined exclusively by the data, no node specificity is needed.

        Returns
        -------
        : list/dict
            The generated parameters from the heuristic model.
        """
        return self.heuristic_model.get_parameters()

    def _assign_constraint_transitions(self, keyframe_node):
        """
        Assigns a unique auto constraint transition string to the keyframe_nodes autoconstraint_transitions field. This is used to determined the constraint transitions akin to the CC-LfD algorithm that generates constraint transition keyframe nodes that cannot be culled.

        Parameters
        ----------
        keyframe_node : dict
            KeyframeGraph node object (dict)
        """
        validity = self._validate_keyframe(keyframe_node)
        keyframe_node["autoconstraint_transitions"].append(self.name + "_" + str(validity))
