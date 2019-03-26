#include "states/state.hpp"
#include "scenarios/scenario.hpp"
#include "scenarios/independent_scenario.hpp"
#include "scenarios/obstacle_scenario.hpp"
#include "strategies/myopic_strategies.hpp"
#include "player.hpp"
#include "ros/ros.h"


using namespace BTP;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wip_BTP");
    ros::NodeHandle n;



    // IndependentBlockageGridScenario scenario(5);
    WallObstacleScenario scenario;
    OptimisticStrategy strat(scenario.getGraph(), scenario.goal);

    Player player(n);

    ros::Duration(1).sleep(); //Sleep to allow publishers to connect

    player.run(scenario, strat, 0.2);


    // viz.vizGraph(scenario.getGraph(), "Grid Graph with Independent Blockage");
    ros::Duration(1).sleep(); //sleep to allow final messages to reach RViz
}
