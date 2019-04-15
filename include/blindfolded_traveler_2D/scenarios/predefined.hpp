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
        ProjectingObstacleBelief bel;
        WallObstacleScenario() :
            ObstacleScenario(Grid(5), 0, 24),
            bel(getGraph(), getLocation())
        {
            name = "Wall Scenario";
            true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.4, -0.1, 0.7, 0.95));
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
        ProjectingObstacleBelief bel;
        std::shared_ptr<Obstacles2D::Rect> true_rect;
        double noise;
        
        SingleWallScenario(std::mt19937& rng, GraphD graph, int start, int goal, double noise) :
            ObstacleScenario(graph, start, goal),
            bel(getGraph(), getLocation()),
            noise(noise)
        {
            name = "Wall_Distribution";
            true_rect = std::make_shared<Obstacles2D::Rect>(0.4, 0.1, 0.7, 1.05);
            true_state.obstacles.obs.push_back(true_rect);
            generateDistribution(rng);
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
                std::uniform_real_distribution<double> rand_offset(-noise, noise);
                double dx = rand_offset(rng);
                double dy = rand_offset(rng);

                auto sampled_rect = std::make_shared<Obstacles2D::Rect>(*true_rect);
                shift(*sampled_rect, dx, dy);
                
                Obstacles2D::Obstacles o;
                o.obs.push_back(sampled_rect);

            
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

    class SparseSingleWallScenario : public SingleWallScenario
    {
    public:
        SparseSingleWallScenario(std::mt19937& rng, double noise) :
            SingleWallScenario(rng, Grid(5), 0, 24, noise)
        {
            name = "Single_Wall_Sparse_noise=" + std::to_string(noise);
        }
    };

    class DenseSingleWallScenario : public SingleWallScenario
    {
    public:
        DenseSingleWallScenario(std::mt19937& rng, double noise) :
            SingleWallScenario(rng, Grid(20), 0, 399, noise)
        {
            name = "Single_Wall_Dense_Graph";
        }
    };
}

#endif
