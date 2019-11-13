# Robot helper user study

This repository contains all the ressources to run the user study and record the data-set

# How to install

```
cd uwds3_ws/src
git clone ...
cd .. & catkin_make
```

# How to play the experiment

To play the experiment and record the data simply do:
```
roslaunch helper_robot run_experiment.launch
```

# How to run only the behavior manager

To run the behavior manager without recording the data do:
```
roslaunch helper_robot behavior_manager.launch
```