#ifndef BTP_OBSTACLE_DISTRIBUTION_HPP
#define BTP_OBSTACLE_DISTRIBUTION_HPP
#include "graph_planner/2d_obstacles.hpp"
#include <random>

namespace BTP
{
    class ObstacleDistribution
    {
    public:
        std::vector<Obstacles2D::Obstacles> o;
        std::vector<double> weights;
        std::vector<double> cum_sum;
    protected:
        double sum = 0;

    public:
        void addElem(Obstacles2D::Obstacles obs, double weight)
        {
            o.push_back(obs);
            weights.push_back(weight);
            sum += weight;
            cum_sum.push_back(sum);
            
        }

        Obstacles2D::Obstacles sample(std::mt19937 &rng) const
        {
            std::uniform_real_distribution<double> dist(0.0, sum);
            double r = dist(rng);

            for(int i=0; i<o.size(); i++)
            {
                if(r <= cum_sum[i])
                {
                    return o[i];
                }
            }
            assert(false && "random number selected out of bounds");
        }
    };
}

#endif
