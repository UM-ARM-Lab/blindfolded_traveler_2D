#include "states/state.hpp"
#include "scenarios/scenario.hpp"
#include "scenarios/independent_scenario.hpp"
#include "scenarios/obstacle_scenario.hpp"
#include "strategies/myopic_strategies.hpp"
#include "player.hpp"
#include "graph_planner/graph_visualization.hpp"
#include "scenarios/predefined.hpp"

#include "ros/ros.h"
#include <gtest/gtest.h>


using namespace BTP;

TEST(basic_behavior, optimistic_wall)
{
    ros::NodeHandle n;
    WallObstacleScenario scenario;
    OptimisticStrategy strat(scenario.getGraph(), scenario.goal);
    Player player(n);
    player.run(scenario, strat);

    EXPECT_GT(scenario.accumulated_cost, 0) << "Reached goal with zero cost";
    EXPECT_LT(scenario.accumulated_cost, 10000) << "Reached goal with way too much cost";

    EXPECT_GT(scenario.edges_attempted, 0) << "Reached goal with not enough edges attempted";
}

TEST(basic_behavior, optimistic_random_wall)
{
    ros::NodeHandle n;
    std::mt19937 rng;
    ManyPossibleWallsScenario scenario(rng);
    OptimisticStrategy strat(scenario.getGraph(), scenario.goal);
    Player player(n);
    player.run(scenario, strat);

    EXPECT_GT(scenario.accumulated_cost, 0) << "Reached goal with zero cost";
    EXPECT_LT(scenario.accumulated_cost, 10000) << "Reached goal with way too much cost";

    EXPECT_GT(scenario.edges_attempted, 0) << "Reached goal with 0 edes attempted";
}


GTEST_API_ int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_basic_behavior");
    return RUN_ALL_TESTS();
}

