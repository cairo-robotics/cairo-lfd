By introducing 1 "gold example" of a new demonstration with a new constraint configuration, the previous demosntrations do not interfere with the correct behavior.

Without explicitly showing this one new demonstration, and simply switching the config files that define the constraints for validation checking of points, the system does not adapt. 

This is likely due to the face that the new demosntration provides just enough data such that the new distributions for each keyframe has enough variation such that correctly constrained keyframe waypoints are sampled.

To run the example, run the model_demosntrations.py using vert_config.json as the config file and using transfer_labeled_demonstrations

One possible benefit to this single gold demonstration for a new constraint definition might be that you can have two different constraint definition work for the same set of demonstrations so long as a "gold" demosntration outlining the constraints is shown for each.