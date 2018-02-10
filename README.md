# lfd
Learnning from demonstration (lfd) packages supporting robotic kinesthetic learning from demonstration for the CAIRO lab at CU-Boulder

## Packages

lfd_modeling - Package containing core libraries and scripts for generating keyframes based on kmeans clustering of trajectories (if so desired), modeling keyframes with Gaussian Mixture Models, ando building graph structure of keyframes representing learned skills and path evaluation results.

lfd_processor_examples - Package containing executable scripts exemplifying some lfd_processor capabilities.

lfd_processor - Package containing core libraries for building LFD environment, I/O, data processsing, and DTW alignment etc,.


## Running lfd_processor_examples

Be sure to install the required PyPi packages to your system (no virtual environment support in ROS *sad_face*).

.../lfd/ $ pip install -r requirements.txt

After building the package in your workspace (watching out for the predicate_classification and predicate_classification_msgs dependencies) run the following:

    $ rosrun lfd_processor_examples gmm_keyframe_example.py
    $ rosrun lfd_processor_examples kmeans_clustering_example.py
