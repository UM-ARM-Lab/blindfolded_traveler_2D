#ifndef BTP_MANY_BOXES_SCENARIO_HPP
#define BTP_MANY_BOXES_SCENARIO_HPP
#include "scenarios/obstacle_scenario.hpp"
#include "beliefs/obstacle_distribution.hpp"
#include <random>
#include "graph_planner/halton_graph.hpp"

namespace BTP
{

    /*************************************
     *** Scenario that has cul-de-sacs ***
     ************************************/
    class ManyBoxesScenario : public ObstacleScenario
    {
    public:
        ProjectingObstacleBelief bel;
        
        ManyBoxesScenario(std::mt19937& rng, GraphD graph, int start, int goal) :
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
            int i=0;
            while(i < 1000)
            {
                // int num_boxes = std::uniform_int_distribution<int>(0, 5)(rng);
                int num_boxes = 10;
                Obstacles2D::Obstacles o;
                for(int box_id=0; box_id<num_boxes; box_id++)
                {
                    std::uniform_real_distribution<double> rand_offset(-0.1, 1.1 - min_obstacle_length);
                    double x = rand_offset(rng);
                    double y = rand_offset(rng);
                    double w = rand_offset(rng)/3 + min_obstacle_length;
                    double h = rand_offset(rng)/3 + min_obstacle_length;
                    o.obs.push_back(std::make_shared<Obstacles2D::Rect>(x, y, x+w, y+h));
                }

                if(!pathExists(o))
                {
                    std::cout << "Rejecting state because no valid path\n";
                    continue;
                }
                i++;
                bel.addElem(o, 1.0);
            }
        }
    };

    class SparseManyBoxesScenario : public ManyBoxesScenario
    {
    public:
        SparseManyBoxesScenario(std::mt19937& rng) :
            ManyBoxesScenario(rng, Grid(5), 0, 24)
        {
            name = "ManyBoxes_Sparse_Graph";
        }
    };

    class DenseManyBoxesScenario : public ManyBoxesScenario
    {
    public:
        DenseManyBoxesScenario(std::mt19937& rng) :
            ManyBoxesScenario(rng, Grid(20), 0, 399)
        {
            name = "ManyBoxes_Dense_Graph";
        }
    };
}

#endif
