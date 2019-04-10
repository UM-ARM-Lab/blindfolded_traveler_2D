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
        ObstacleBelief bel;
        
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

        bool pathExists(Obstacles2D::Obstacles &o)
        {
            ObstacleState s(&graph, true_state.current_location, o);

            using namespace arc_dijkstras;

            const auto distance_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    return edge.getWeight();
                };

            const auto edge_validity_check_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    return s.getBlockage(edge.getFromIndex(), edge.getToIndex()) >= 1;
                };

            auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformLazyAstar(
                graph, true_state.current_location, goal,
                edge_validity_check_fn,
                distance_fn, 
                &distanceHeuristic, true);
            if(result.second >= std::numeric_limits<double>::max())
            {
                std::cout << "Rejecting state because no valid path\n";
                return false;
            }
            return true;

        }
        
        void generateDistribution(std::mt19937 &rng)
        {
            int i=0;
            while(i < 1000)
            {
                int num_boxes = std::uniform_int_distribution<int>(0, 5)(rng);
                
                Obstacles2D::Obstacles o;
                for(int box_id=0; box_id<num_boxes; box_id++)
                {
                    std::uniform_real_distribution<double> rand_offset(0, 1.0);
                    double x = rand_offset(rng);
                    double y = rand_offset(rng);
                    double w = rand_offset(rng)/3;
                    double h = rand_offset(rng)/3;
                    o.obs.push_back(std::make_shared<Obstacles2D::Rect>(x, y, x+w, y+h));
                }

                if(!pathExists(o))
                {
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
