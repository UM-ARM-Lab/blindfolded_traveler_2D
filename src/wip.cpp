#include "scenarios/predefined.hpp"
#include "strategies/myopic_strategies.hpp"
#include "strategies/optimistic_rollout.hpp"
#include "scenarios/obstacle_scenario.hpp"
#include "player.hpp"
#include "ros/ros.h"

using namespace BTP;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wip_BTP");
    ros::NodeHandle n;
    std::mt19937 rng;
    rng.seed(time(0));


    // IndependentBlockageGridScenario scenario(5);
    // WallObstacleScenario scenario;
    // ManyPossibleWallsScenario scenario;
    // OptimisticStrategy strat(scenario.getGraph(), scenario.goal);

    
    // Obstacles2D::Obstacles o;
    // o.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.4, -0.1, 0.6, 0.95));
    // ObstacleState true_state(Grid(5), 0, o);
    
    // ObstacleScenario scenario(true_state, 24);
    // OmniscientStrategy strat(true_state, 24);

    ManyPossibleWallsScenario scenario(rng);
    // BestExpectedStrategy strat(scenario.getGraph(), scenario.goal, scenario.bel);
    // OptimisticStrategy strat(scenario.getGraph(), scenario.goal);
    OptimisticRollout strat(scenario.getGraph(), scenario.goal, scenario.bel);
    

    Player player(n);

    ros::Duration(1).sleep(); //Sleep to allow publishers to connect

    player.run(scenario, strat, 0.2);


    // viz.vizGraph(scenario.getGraph(), "Grid Graph with Independent Blockage");
    ros::Duration(1).sleep(); //sleep to allow final messages to reach RViz
}
