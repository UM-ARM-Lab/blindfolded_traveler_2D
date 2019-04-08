# blindfolded_traveler_2D

[![Build Status](https://travis-ci.com/UM-ARM-Lab/blindfolded_traveler_2D.svg?branch=master)](https://travis-ci.com/UM-ARM-Lab/blindfolded_traveler_2D)


## Purpose
This explores the "Blindfolded Traveler's Problem". The code is still in the early stages, and the interfaces are not stable

## Installation
There is Continuous Integration set up using TravisCI. Follow the docker image and `setup.sh` script for the latest dependencies.
As an overview, this depends on:

1. ROS (tested on kinetic). Organize repos in the catkin format, within a catkin workspace. Pull each git repo within `catkin_ws/src`
1. https://github.com/UM-ARM-Lab/arc_utilities/tree/CleanUpDijkstras (be sure to get the correct branch)
2. https://github.com/UM-ARM-Lab/unknown_graph_planner/tree/BlindfoldedTraveler (be sure to get the correct branch)
3. flann: `sudo apt install libpcl-dev`

## Usage
1. Build. I use `catkin build` available in catkin-python-tools, though `catkin_make_isolated` should work as well
1. `roscore`
2. In a new terminal run `rviz` and open the `graph.rviz` configuration
2. In a new terminal run `rosrun blind_traveler_2D wip`
4. Watch the rviz graph and agent update


For running all trials, `cd blindfolded_traaveler_2D/experiments` and `rosrun blind_traveler_2D run_trials`

## Overview

The Blindfolded Traveler's Problem is where traveler on a graph is attempting to reach a goal node, however the traveler does not learn the validity of an edge until attempted. 

**Scenario**: A Scenario contain the true state of the world, including the graph, the agent location, the goal, and the blockages of any edges. A scenario also stores the full set of observations a travelers has seen.

**Strategy**: A strategy takes in the graph (with unknown edge validity) and all past observations and outputs an action.
