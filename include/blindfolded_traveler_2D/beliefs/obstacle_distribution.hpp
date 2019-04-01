#ifndef BTP_OBSTACLE_DISTRIBUTION_HPP
#define BTP_OBSTACLE_DISTRIBUTION_HPP
#include "graph_planner/2d_obstacles.hpp"
#include "graph_planner/graph_visualization.hpp"
#include "states/state.hpp"
#include "observations.hpp"
#include <random>

namespace BTP
{

    inline bool isConsistent(const Observation& obs, const ObstacleState& s)
    {
        Observation should(obs.from, obs.to, s.getBlockage(obs.from, obs.to));
        return should.succeeded() == obs.succeeded();
    }


    typedef std::pair<std::shared_ptr<State>, double> WeightedState;

    

    class ObstacleBelief
    {
    public:
        GraphD graph;
        Location cur;
        
        std::vector<Obstacles2D::Obstacles> o;
        std::vector<double> weights;
        std::vector<double> cum_sum;
    protected:
        double sum = 0;

    public:
        ObstacleBelief(GraphD graph, Location cur) :
            graph(graph), cur(cur)
        {}
        
        void addElem(Obstacles2D::Obstacles obs, double weight)
        {
            o.push_back(obs);
            weights.push_back(weight);
            sum += weight;
            cum_sum.push_back(sum);
            
        }

        // Obstacles2D::Obstacles sample(std::mt19937 &rng) const
        std::shared_ptr<ObstacleState> sampleObstacleState(std::mt19937 &rng) const
        {
            std::uniform_real_distribution<double> dist(0.0, sum);
            double r = dist(rng);

            for(int i=0; i<o.size(); i++)
            {
                if(r <= cum_sum[i])
                {
                    return std::make_shared<ObstacleState>(&graph, cur, o[i]);
                }
            }
            assert(false && "random number selected out of bounds");
        }

        std::shared_ptr<State> sample(std::mt19937 &rng) const
        {
            return std::static_pointer_cast<State>(sampleObstacleState(rng));
        }

        std::vector<WeightedState> getWeightedStates() const
        {
            std::vector<WeightedState> ws;
            for(int i=0; i<o.size(); i++)
            {
                ws.push_back(std::make_pair(std::static_pointer_cast<State>(
                                                std::make_shared<ObstacleState>(&graph, cur, o[i])),
                                            weights[i]));
            }
            return ws;
        }

        void markInvalidEnvironments(Observation obs)
        {
            for(int i=0; i<o.size(); i++)
            {
                if(weights[i] == 0)
                {
                    continue;
                }
                ObstacleState h(&graph, obs.from, o[i]);
                if(!isConsistent(obs, h))
                {
                    weights[i] = 0;
                }
            }

            sum = 0;
            for(int i=0; i<weights.size(); i++)
            {
                sum += weights[i];
                cum_sum[i] = sum;
            }
        }

        void setLocation(Location loc)
        {
            cur = loc;
        }

        Location getLocation() const
        {
            return cur;
        }
        
        void update(Observation obs)
        {
            using namespace arc_dijkstras;
            GraphEdge& e = graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID);
            markInvalidEnvironments(obs);
        }


        void viz(GraphVisualizer &viz) const
        {
            Obstacles2D::Obstacles full_belief;
            for(int i=0; i<o.size(); i++)
            {
        
                for(const auto& obstacle: o[i].obs)
                {
                    if(weights[i] == 0)
                    {
                        full_belief.obs.push_back(std::make_shared<Obstacles2D::Empty>());
                        continue;
                    }
                    full_belief.obs.push_back(obstacle);
                }
            }
            viz.vizObstacles(full_belief, 0.01, "Belief", "clear red");
        }
    };
}

#endif
