# lfd
Learnning from demonstration (lfd) packages supporting robotic kinesthetic learning from demonstration.

## Packages

lfd_keyframe - Package containing core libraries and scripts for generating keyframes based on kmeans clustering of trajectories (if so desired) and modeling keyframes with Gaussian Mixture Models.

lfd_keyframe_examples - Package containing executable scripts exemplifying the lfd_keyframe capabilities (with visualizations)

lfd_network - Package containing core libraries for building graph structure of keyframes representing learned skills and path evaluation results.


## Running lfd_keyframe_examples

Be sure to install the required PyPi packages to your system (no virtual environment support in ROS *sad_face*).

.../lfd/ $ pip install -r requirements.txt

After building the package in your workspace (watching out for the predicate_classification and predicate_classification_msgs dependenciers) run the following:

    $ rosrun lfd_example_examples gmm_keyframe_example.py
    $ rosrun lfd_example_examples kmeans_clustering_example.py
