#include "states/state.hpp"
#include "scenarios/scenario.hpp"
#include "scenarios/independent_scenario.hpp"
#include "strategies/myopic_strategies.hpp"
#include "player.hpp"
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
    OptimisticStrategy strat(scenario.getGraph(), scenario.goal);

    Player player;

    player.run(scenario, strat);


    viz.vizGraph(scenario.getGraph(), "Grid Graph with Independent Blockage");
}
