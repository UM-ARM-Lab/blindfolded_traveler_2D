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
    SparseSingleWallScenario scenario(rng);
    OptimisticStrategy strat(scenario.getGraph(), scenario.goal);
    Player player(n);
    player.run(scenario, strat);

    EXPECT_GT(scenario.accumulated_cost, 0) << "Reached goal with zero cost";
    EXPECT_LT(scenario.accumulated_cost, 10000) << "Reached goal with way too much cost";

    EXPECT_GT(scenario.edges_attempted, 0) << "Reached goal with 0 edes attempted";
}

TEST(basic_behavior, obstacle_distribution_likelihood)
{
    GraphD g = Grid(2);
    ObstacleBelief ob(g, 0);
    {
        Obstacles2D::Obstacles o;
        o.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.4, -.1, 0.49, 0.5));
        ob.addElem(o, 3.0);
    }
    {
        Obstacles2D::Obstacles o;
        o.obs.push_back(std::make_shared<Obstacles2D::Rect>(-0.4, -.1, -0.5, 0.5));
        ob.addElem(o, 1.0);
    }

    std::vector<Observation> blocked{
        {0, 2, 1.0},
        {0, 3, 1.0}};

    for(const auto& u: blocked)
    {
        EXPECT_EQ(ob.getLikelihood(u), 1.0/4.0) <<
            "Expected obstacle to blocked edge (" << u.from << ", " << u.to << ") 3/4 of the time";
    }

    std::vector<Observation> unblocked{
        {0, 1, 1.0},
        {1, 3, 1.0},
        {2, 3, 1.0},
        {1, 2, 1.0}};
    for(const auto& u: unblocked)
    {
        EXPECT_EQ(ob.getLikelihood(u), 1.0) <<
            "Expected edge (" << u.from << ", " << u.to << ") to be unblocked";
    }
}



GTEST_API_ int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_basic_behavior");
    return RUN_ALL_TESTS();
}

