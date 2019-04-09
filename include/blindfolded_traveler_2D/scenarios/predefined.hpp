#ifndef BTP_PREDEFINED_SCENARIO_HPP
#define BTP_PREDEFINED_SCENARIO_HPP
#include "scenarios/obstacle_scenario.hpp"
#include "beliefs/obstacle_distribution.hpp"
#include <random>
#include "graph_planner/halton_graph.hpp"


namespace BTP
{
    class WallObstacleScenario : public ObstacleScenario
    {
    public:
        ObstacleBelief bel;
        WallObstacleScenario() :
            ObstacleScenario(Grid(5), 0, 24),
            bel(getGraph(), getLocation())
        {
            name = "Wall Scenario";
            true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.4, -0.1, 0.6, 0.95));
            bel.addElem(true_state.obstacles, 1.0);
        }

        virtual const Belief& getPrior() const override
        {
            return bel;
        }
    };

    class SingleWallScenario : public ObstacleScenario
    {
    public:
        ObstacleBelief bel;
        
        SingleWallScenario(std::mt19937& rng, GraphD graph, int start, int goal) :
            ObstacleScenario(graph, start, goal),
            bel(getGraph(), getLocation())
        {
            name = "Wall_Distribution";
            generateDistribution(rng);
            true_state = *bel.sampleObstacleState(rng);
        }

        virtual const Belief& getPrior() const override
        {
            return bel;
        }

    private:
        void generateDistribution(std::mt19937 &rng)
        {
            for(int i=0; i<1000; i++)
            {
                std::uniform_real_distribution<double> rand_offset(-0.2, 0.2);
                double dx = rand_offset(rng);
                double dy = rand_offset(rng);
                Obstacles2D::Obstacles o;
                o.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.4 + dx, -0.1 + dy,
                                                                    0.6 + dx, 0.85 + dy));
                bel.addElem(o, 1.0);
            }
        }
    };

    class SparseSingleWallScenario : public SingleWallScenario
    {
    public:
        SparseSingleWallScenario(std::mt19937& rng) :
            SingleWallScenario(rng, Grid(5), 0, 24)
        {
            name = "Single_Wall_Sparse_Graph";
        }
    };

    class DenseSingleWallScenario : public SingleWallScenario
    {
    public:
        DenseSingleWallScenario(std::mt19937& rng) :
            SingleWallScenario(rng, Grid(20), 0, 399)
        {
            name = "Single_Wall_Dense_Graph";
        }
    };
}

#endif
