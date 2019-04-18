#ifndef BTP_INDEP_EDGE_BELIEF_HPP
#define BTP_INDEP_EDGE_BELIEF_HPP
#include "beliefs/belief.hpp"
#include "beliefs/obstacle_distribution.hpp"

namespace BTP
{
    class IndepEdgeBelief : public Belief
    {
    public:
        GraphD graph;
        Location cur;
        std::map<arc_dijkstras::HashableEdge, double> prob_free;

        IndepEdgeBelief(const GraphD& graph, Location cur,
                        std::map<arc_dijkstras::HashableEdge, double> prob_free) :
            graph(graph), cur(cur), prob_free(prob_free)
        {}
        
        IndepEdgeBelief(const ObstacleBelief& b) :
            graph(b.graph), cur(b.cur)
        {
            for(const auto& n: graph.getNodes())
            {
                for(const auto& e: n.getOutEdges())
                {
                    Observation obs(e.getFromIndex(), e.getToIndex(), 1.0);
                    prob_free[arc_dijkstras::getHashable(e)] = b.getLikelihood(obs);
                }
            }
        }


        virtual std::unique_ptr<State> sample(std::mt19937 &rng) const override
        {
            std::set<arc_dijkstras::HashableEdge> blocked_edges;
                
            std::uniform_real_distribution<double> d(0.0, 1.0);
            for(const auto& edge_prob: prob_free)
            {
                if(edge_prob.second < d(rng))
                {
                    blocked_edges.insert(edge_prob.first);
                }
            }
            return std::make_unique<IndependentBlockageState>(&graph, cur, blocked_edges);
        }
        
        virtual double getLikelihood(Observation obs) const override
        {
            arc_dijkstras::HashableEdge e(obs.from, obs.to);
            if(obs.blockage >= 1.0)
            {
                return prob_free.at(e);
            }
            return 1.0 - prob_free.at(e);
        }
        
        virtual std::unique_ptr<Belief> clone() const override
        {
            return std::make_unique<IndepEdgeBelief>(graph, cur, prob_free);
        }
        
        virtual void update(Observation obs) override
        {
            arc_dijkstras::HashableEdge e(obs.from, obs.to);
            if(obs.blockage < 1.0)
            {
                prob_free[e] = 0.0;
                return;
            }
            prob_free[e] = 1.0;
            cur = obs.to;
        }
        
        
        virtual void viz(GraphVisualizer &viz) const
        {}
        
        virtual std::string getName() const override
        {
            return "IndepEdge";
        }
    };
}

#endif


        
