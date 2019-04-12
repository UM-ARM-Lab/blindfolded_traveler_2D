#include "beliefs/projection.hpp"
#include "states/state.hpp"
#include <gtest/gtest.h>

using namespace BTP;
using namespace Obstacles2D;

TEST(ProjectionTest, getNearest_returns_nearest_obstacle)
{
    Obstacles obstacles;
    auto r1 = std::make_shared<Rect>(0, 0, 1, 1);
    auto r2 = std::make_shared<Rect>(2, 0, 3, 1);

    obstacles.obs.push_back(r1);
    obstacles.obs.push_back(r2);


    EXPECT_EQ(getNearest(obstacles, std::vector<double>{0, 0}), r1);
    EXPECT_EQ(getNearest(obstacles, std::vector<double>{5, 4}), r2);
}

TEST(ProjectionTest, project_rectancle_onto_side_wall_yeilds_state_consistent_with_observation)
{
    Obstacles obstacles;
    obstacles.obs.push_back(std::make_shared<Rect>(0.7, 0.2, 1.0, 0.8));
    Grid g(2);
    ObstacleState s(&g, 0, obstacles);

    {
        Observation z(0, 3, 0.5);
        EXPECT_NE(s.getBlockage(z.from, z.to), z.blockage) << "Blockage already matches in setup: Test invalid";
        makeConsistent(s, z);
        EXPECT_EQ(s.getBlockage(z.from, z.to), z.blockage) << "Blockage does not match after makeConsistent";
    }
    {
        Observation z(3, 0, 0.5);
        EXPECT_NE(s.getBlockage(z.from, z.to), z.blockage) << "Blockage already matches in setup: Test invalid";
        makeConsistent(s, z);
        EXPECT_EQ(s.getBlockage(z.from, z.to), z.blockage) << "Blockage does not match after makeConsistent";
    }
}

TEST(ProjectionTest, project_rectancle_onto_top_bottom_wall_yeilds_state_consistent_with_observation)
{
    Obstacles obstacles;
    obstacles.obs.push_back(std::make_shared<Rect>(0.2, 0.7, 0.8, 1.0));
    Grid g(2);
    ObstacleState s(&g, 0, obstacles);

    {
        Observation z(0, 3, 0.5);
        EXPECT_NE(s.getBlockage(z.from, z.to), z.blockage) << "Blockage already matches in setup: Test invalid";
        makeConsistent(s, z);
        EXPECT_EQ(s.getBlockage(z.from, z.to), z.blockage) << "Blockage does not match after makeConsistent";
    }
    {
        Observation z(3, 0, 0.5);
        EXPECT_NE(s.getBlockage(z.from, z.to), z.blockage) << "Blockage already matches in setup: Test invalid";
        makeConsistent(s, z);
        EXPECT_EQ(s.getBlockage(z.from, z.to), z.blockage) << "Blockage does not match after makeConsistent";
    }
}

TEST(ProjectionTest, project_rectangle_onto_corner_yeilds_state_consistent_with_observation)
{
    Obstacles obstacles;
    obstacles.obs.push_back(std::make_shared<Rect>(3, 3, 4, 4));
    Grid g(2);
    ObstacleState s(&g, 0, obstacles);
    
    {
        Observation z(0, 3, 0.5);
        EXPECT_NE(s.getBlockage(z.from, z.to), z.blockage) << "Blockage already matches in setup: Test invalid";
        EXPECT_EQ(s.getBlockage(z.from, z.to), 1.0);
        makeConsistent(s, z);
        EXPECT_EQ(s.getBlockage(z.from, z.to), z.blockage) << "Blockage does not match after makeConsistent";
    }
    
}



GTEST_API_ int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
