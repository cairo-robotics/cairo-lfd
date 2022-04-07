# Peg-in-hole Task

## Summary

This task is intended to show that a CC-LfD keyframe model using a constrained planner can more efficiently cope with maintaining the correct orientation for a peg-in-hole problem. Demonstration data will be used to boost efficiency, but the idea is to show that the regular PRM requires more points and collisions checks in total than does the constrained PRM to insert the peg in the hole in the correct orientation given the keyframe segment. Likewise, we can show that the keyframe models themselves using naive motion planners will not maintain the orientation constraint consistently enough to result in high levels of sklil success.

## Constraints

There will be one single orientation constraint in this case. The idea is to be able to ramp up number of keyframes required to show that dense model is required for any sort of success. Then by sparsifying the number of keyframes, reliance on constrained motion planning should allow for better skill success. 

## Evaluation

This is mostly a proof of concept evaluation, but we can try to run a varying state-of-the-art motion planners via OMPL and Moveit to show that most planners, regardless of their efficiency and optimiality criteria, will still fail to adhere to constraints while at the same time solving the demonstrated task. Likewise, ramping up number of keyframes is the only way for these constrained motion planners to not fail at the skill.

We should also quantitatively evaluated the number of collision checks required: see LazyPRM paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=844107