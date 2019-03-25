#include "states/state.hpp"
#include "scenarios/scenario.hpp"
#include "scenarios/independent_scenario.hpp"
#include "ros/ros.h"
#include "graph_planner/graph_visualization.hpp"


using namespace BTP;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wip_BTP");
    ros::NodeHandle n;

    GraphVisualizer viz(n);

    ros::Duration(1).sleep();

    IndependentBlockageGridScenario scenario(5);


    viz.vizGraph(scenario.getGraph(), "Grid Graph with Independent Blockage");
}
