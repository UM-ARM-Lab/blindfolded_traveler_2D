#ifndef BTP_OBSTACLE_DISTRIBUTION_HPP
#define BTP_OBSTACLE_DISTRIBUTION_HPP
#include "graph_planner/2d_obstacles.hpp"
#include "graph_planner/graph_visualization.hpp"
#include "states/state.hpp"
#include "observations.hpp"
#include "beliefs/belief.hpp"
#include <random>

namespace BTP
{

    inline bool isConsistent(const Observation& obs, const ObstacleState& s)
    {
        Observation should(obs.from, obs.to, s.getBlockage(obs.from, obs.to));
        return should.succeeded() == obs.succeeded();
    }




    

    class ObstacleBelief : public ExplicitBelief
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

        virtual std::unique_ptr<Belief> clone() const override
        {
            return cloneExplicit();
        }

        virtual std::unique_ptr<ExplicitBelief> cloneExplicit() const override
        {
            std::unique_ptr<ObstacleBelief> b = std::make_unique<ObstacleBelief>(graph, cur);
            b->o = o;
            b->weights = weights;
            b->cum_sum = cum_sum;
            b->sum = sum;
            return b;
        }
        
        void addElem(Obstacles2D::Obstacles obs, double weight)
        {
            o.push_back(obs);
            weights.push_back(weight);
            sum += weight;
            cum_sum.push_back(sum);
        }

        std::unique_ptr<ObstacleState> sampleObstacleState(std::mt19937 &rng) const
        {
            std::uniform_real_distribution<double> dist(0.0, sum);
            double r = dist(rng);

            for(int i=0; i<o.size(); i++)
            {
                if(r <= cum_sum[i])
                {
                    return std::make_unique<ObstacleState>(&graph, cur, o[i]);
                }
            }
            assert(false && "random number selected out of bounds");
        }

        virtual std::unique_ptr<State> sample(std::mt19937 &rng) const override
        {
            return sampleObstacleState(rng);
        }

        virtual std::vector<WeightedState> getWeightedStates() const override
        {
            std::vector<WeightedState> ws;
            for(int i=0; i<o.size(); i++)
            {
                ws.push_back(std::make_pair(std::make_shared<ObstacleState>(&graph, cur, o[i]),
                                            weights[i]));
            }
            return ws;
        }

        virtual double getLikelihood(Observation obs) const
        {
            double agreement = 0;
            for(const auto& ws: getWeightedStates())
            {
                if(ws.first->getBlockage(obs.from, obs.to) == obs.blockage)
                {
                    agreement += ws.second;
                }
            }
            
            return agreement / sum;
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

        virtual void update(Observation obs) override
        {
            using namespace arc_dijkstras;
            GraphEdge& e = graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID);
            markInvalidEnvironments(obs);

            if(obs.succeeded())
            {
                cur = obs.to;
            }
        }


        void viz(GraphVisualizer &viz) const override
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
            auto color = colorLookup("clear red");
            color.a = std::max(1.0/sum, 1.0/255);
            viz.vizObstacles(full_belief, 0.01, "Belief", color);
        }

        virtual std::string getName() const override
        {
            return "ObstaclePrior";
        }
    };
}

#endif
