# Glue Application Task

## Summary

This task is intended to show that a CC-LfD keyframe model can serve as the basis for a changing constraint set planning problem. It also intends to show that altering one constraint allows you to retain the the trajectory portions of segments unaffected by the alteration, only requiring a replanning phase for the affected segments. The robot will be applying "glue" along three different lines across the table. What will change is the orientation required for the middle line. These lines will be staggered so that the middle line is further away from the robot. In essence the robot is moving from line to line, tracing along each as it applies glue. The idea is that by changing the middle lines orientation constrain, I can show that the planed path that we spline through to generate a robot trajectory only has to be replanned for that segment and the segment leading up to it.

## Constraints

This task will have three different sets of orientational and two positional constraints. The two positional/planar constraints will force it to trace along the table within a relatively narrow vertical band, and the orientation will restrict that angle of the application. 

## Evaluation

This is mostly a proof of concept evaluation. In essence, I want to show that changing the single constraint on how glue must be applied to middle line only effects two segments. The robot behavior should almost be identical leading up to it, but only change for that one leadup segment and application segment.