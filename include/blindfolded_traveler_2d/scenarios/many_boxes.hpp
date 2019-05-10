#ifndef BTP_MANY_BOXES_SCENARIO_HPP
#define BTP_MANY_BOXES_SCENARIO_HPP
#include "scenarios/obstacle_scenario.hpp"
#include "beliefs/obstacle_distribution.hpp"
#include <random>
#include "graph_planner/halton_graph.hpp"
#include "arc_utilities/pretty_print.hpp"

namespace BTP
{

    /*************************************
     *** Scenario that has cul-de-sacs ***
     ************************************/
    class ManyBoxesScenario : public ObstacleScenario
    {
    public:
        ProjectingObstacleBelief bel;
        double noise;
        const std::vector<double> bias;
        
        ManyBoxesScenario(std::mt19937& rng, GraphD graph, int start, int goal, double noise,
                          const std::vector<double>& bias = {0,0}) :
            ObstacleScenario(graph, start, goal),
            bel(getGraph(), getLocation()),
            noise(noise),
            bias(bias)
        {
            name = "Wall_Distribution";
            makeTrueState();
            generateDistribution(rng);
        }

        virtual const Belief& getPrior() const override
        {
            return bel;
        }

    private:

        void makeTrueState()
        {
            true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.24, 0.4, 0.47, 0.76));
            true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.24, 0.74, 0.51, 1.02));
            true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.35, 0.4, 0.6, 0.6));
            true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.4, 0.4, 0.8, 0.6));
            true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.27, 0.1, 0.48, 0.4));
            true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.9, -0.1, 1.1, 0.1));
            // true_state.obstacles.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.1, 0.4, 0.2, 0.5));
        }
        
        void generateDistribution(std::mt19937 &rng)
        {
            using namespace Obstacles2D;
            int i=0;
            while(i < 1000)
            {
                // int num_boxes = std::uniform_int_distribution<int>(0, 5)(rng);
                // int num_boxes = 10;
                Obstacles particle;
                // for(int box_id=0; box_id<num_boxes; box_id++)
                // {
                //     std::uniform_real_distribution<double> rand_offset(-0.1, 1.1 - min_obstacle_length);
                //     double x = rand_offset(rng);
                //     double y = rand_offset(rng);
                //     double w = rand_offset(rng)/3 + min_obstacle_length;
                //     double h = rand_offset(rng)/3 + min_obstacle_length;
                //     o.obs.push_back(std::make_shared<Obstacles2D::Rect>(x, y, x+w, y+h));
                // }
                
                for(auto true_obstacle: true_state.obstacles.obs)
                {
                    auto noisy_rect = std::make_shared<Rect>(*dynamic_cast<Rect*>(true_obstacle.get()));
                    std::normal_distribution<double> rand_offset(0.0, noise);
                    double dx = rand_offset(rng) + bias[0];
                    double dy = rand_offset(rng) + bias[1];
                    shift(*noisy_rect, dx, dy);
                    particle.obs.push_back(noisy_rect);
                }

                if(!pathExists(particle))
                {
                    std::cout << "Rejecting state because no valid path\n";
                    continue;
                }
                i++;
                bel.addElem(particle, 1.0);
            }
        }
    };

    class SparseManyBoxesScenario : public ManyBoxesScenario
    {
    public:
        SparseManyBoxesScenario(std::mt19937& rng, double noise, const std::vector<double>& bias = {0,0}) :
            ManyBoxesScenario(rng, Grid(5), 0, 24, noise, bias)
        {
            name = "ManyBoxes_noise=" + PrettyPrint::PrettyPrint(noise) +
                "_bias=(" + PrettyPrint::PrettyPrint(bias) + ")";
        }
    };

    class DenseManyBoxesScenario : public ManyBoxesScenario
    {
    public:
        DenseManyBoxesScenario(std::mt19937& rng, double noise, const std::vector<double>& bias = {0,0}) :
            ManyBoxesScenario(rng, Grid(20), 0, 399, noise, bias)
        {
            name = "ManyBoxes_Dense_Graph_noise=" + std::to_string(noise);
        }
    };
}

#endif
