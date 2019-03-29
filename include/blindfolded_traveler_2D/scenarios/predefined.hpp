#ifndef BTP_PREDEFINED_SCENARIO_HPP
#define BTP_PREDEFINED_SCENARIO_HPP
#include "scenarios/obstacle_scenario.hpp"
#include "distributions/obstacle_distribution.hpp"
#include <random>
#include "graph_planner/halton_graph.hpp"


namespace BTP
{
    class WallObstacleScenario : public ObstacleScenario
    {
    public:
        WallObstacleScenario() :
            ObstacleScenario(Grid(5), 0, 24)
        {
            name = "Wall Scenario";
            true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.4, -0.1, 0.6, 0.95));
        }

    };

    class ManyPossibleWallsScenario : public ObstacleScenario
    {
    public:
        ObstacleDistribution d;
        
        ManyPossibleWallsScenario() :
            ObstacleScenario(Grid(5), 0, 24)
        {
            name = "Wall Distribution";
                
            std::mt19937 rng;
            rng.seed(time(0));

            generateDistribution(rng);
            true_state.obstacles = d.sample(rng);
        }

    private:
        void generateDistribution(std::mt19937 &rng)
        {
            for(int i=0; i<100; i++)
            {
                std::uniform_real_distribution<double> rand_offset(-0.2, 0.2);
                double dx = rand_offset(rng);
                double dy = rand_offset(rng);
                Obstacles2D::Obstacles o;
                o.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.4 + dx, -0.1 + dy,
                                                                    0.6 + dx, 0.85 + dy));
                d.addElem(o, 1.0);
            }
        }
    };
}

#endif
