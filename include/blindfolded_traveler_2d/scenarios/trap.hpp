#ifndef BTP_TRAP_SCENARIO_HPP
#define BTP_TRAP_SCENARIO_HPP
#include "scenarios/obstacle_scenario.hpp"
#include "beliefs/obstacle_distribution.hpp"
#include <random>
#include "graph_planner/halton_graph.hpp"

namespace BTP
{

    /*************************************
     *** Scenario that has cul-de-sacs ***
     ************************************/
    class TrapScenario : public ObstacleScenario
    {
    public:
        ObstacleBelief bel;
        
        TrapScenario(std::mt19937& rng, GraphD graph, int start, int goal) :
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
                std::uniform_real_distribution<double> rand_offset(-0.1, 0.1);
                double dx = rand_offset(rng);
                double dy = rand_offset(rng);
                Obstacles2D::Obstacles o;
                o.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.65 + dx, 0.12 + dy,
                                                                    0.85 + dx, 1.12 + dy));
                o.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.45 + dx, 0.6 + dy,
                                                                    0.85 + dx, 0.8 + dy));
                bel.addElem(o, 1.0);
            }
        }
    };

    class SparseTrapScenario : public TrapScenario
    {
    public:
        SparseTrapScenario(std::mt19937& rng) :
            TrapScenario(rng, Grid(5), 0, 24)
        {
            name = "Trap_Sparse_Graph";
        }
    };

    class DenseTrapScenario : public TrapScenario
    {
    public:
        DenseTrapScenario(std::mt19937& rng) :
            TrapScenario(rng, Grid(20), 0, 399)
        {
            name = "Trap_Dense_Graph";
        }
    };
}

#endif
