# blindfolded_traveler_2D

[![Build Status](https://travis-ci.com/UM-ARM-Lab/blindfolded_traveler_2D.svg?branch=master)](https://travis-ci.com/UM-ARM-Lab/blindfolded_traveler_2D)


## Purpose
This explores the "Blindfolded Traveler's Problem". The code is still in the early stages, and the interfaces are not stable

## Installation
There is Continuous Integration set up using TravisCI. Follow the docker image and `setup.sh` script for the latest dependencies.
As an overview, this depends on:

1. ROS (tested on kinetic)
1. https://github.com/UM-ARM-Lab/arc_utilities/tree/CleanUpDijkstras (be sure to get the correct branch)
2. https://github.com/UM-ARM-Lab/unknown_graph_planner/tree/BlindfoldedTraveler (be sure to get the correct branch)
3. flann: `sudo apt install libpcl-dev`

## Overview

The Blindfolded Traveler's Problem is where traveler on a graph is attempting to reach a goal node, however the traveler does not learn the validity of an edge until attempted. 

**Scenario**: A Scenario contain the true state of the world, including the graph, the agent location, the goal, and the blockages of any edges. A scenario also stores the full set of observations a travelers has seen.

**Strategy**: A strategy takes in the graph (with unknown edge validity) and all past observations and outputs an action.
