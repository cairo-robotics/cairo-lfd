# import rospy

# from cairo_planning.constraints.optimization import TSROptimization, OmegaTSROptimization


# class TSROptimizer():
    
#     def __init__(self, tsr, constraint_ids, settings_fp=None):
#         self.constraint_ids = constraint_ids
#         self.optimizer = TSROptimization(settings_fp, tsr, optimization_iterations=1000)
    
#     def optimize(self, keyframe_point):
#         return self.optimizer.optimize(keyframe_point)

# class OmegaTSROptimizer():
    
#     def __init__(self, tsr, constraint_ids, settings_fp=None):
#         self.constraint_ids = constraint_ids
#         self.optimizer = OmegaTSROptimization(settings_fp, tsr, optimization_iterations=1000)
    
#     def optimize(self, keyframe_point, seed_point):
#         return self.optimizer.optimize(seed_point, keyframe_point)


# class OptimizerFactory(object):
#     """
#     Factory class that builds LFD constraints. These items are defined in the config.json file.
#     The class field in the configuration determines which constraint class to use.

#     Attributes
#     ----------
#     configs : list
#             List of configuration dictionaries.
#     classes : dict
#         Dictionary with values as uninitialized class references to constraint classes i.e. HeightConstraint

#     Example
#     -------

#     Example entry in config.json:

#     .. code-block:: json

#         {
#             "class": "HeightConstraint",
#             "init_args" :
#                 {
#                     "id": 1,
#                     "item": 1,
#                     "button": "right_button_square",
#                     "reference_height": 0.0,
#                     "threshold_distance": 0.25
#                 }
#         }
#     """
#     def __init__(self, configs):
#         """
#         Parameters
#         ----------
#         configs : list
#             List of configuration dictionaries.
#         """
#         self.configs = configs
#         self.optimizer_classes = {
#             "TSROptimizer": TSROptimizer,
#             "OmegaTSROptimizer": OmegaTSROptimizer
#         }

#     def generate_optimizers(self):
#         """
#         Build the constraint objects defined in the configuration dictionaries of self.configs.

#         Returns
#         -------
#         robots : list
#             List of constraint objects.
#         """
#         optimizers = []
#         for config in self.configs:
#             try:
#                 optimizers.append(self.optimizer_classes[config["class"]](**config["init_args"]))
#             except TypeError as e:
#                 rospy.logerr("Error constructing {}: {}".format(self.optimizer_classes[config["class"]], e))
#         return optimizers
