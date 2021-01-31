# Carrying Task

## Summary

This task is intended to show that for CC-LfD to successfully cope with keyframe occlusion it must integrate constrained motion planning. Without constrained motion planning methods, keyframe occlusion recludes the reliance on a course constrained trajectory of waypoints, push more of robot execution reliance onto motion planning. Naive motion planning has no ability to adhere to constraints.

## Constraints

This task will have two constraints. An upright orientation constraint, and a planar/height constraint that will restrict the arm from being able to simply go over an object. The object will be placed such that it is feasible for the robot to move through the space in front of the object while still maintaining the upright constraint.

Upright Orientation RPY: 1.79422024839, 1.50645900879, 1.46489904133

## Evaluation

This is mostly a proof of concept evaluation, but we can try to run a varying state-of-the-art motion planners via OMPL and Moveit to show that most planners, regardless of their efficiency and optimiality criteria, will still fail to adhere to constraints while at the same time solving the demonstrated task. 

We should also quantitatively evaluated the number of collision checks required: see LazyPRM paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=844107