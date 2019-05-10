#ifndef BTP_MYOPIC_STRATEGY_HPP
#define BTP_MYOPIC_STRATEGY_HPP

#include "strategies/strategy.hpp"
#include "graph_planner/a_star.hpp"
#include "arc_utilities/eigen_helpers.hpp"
#include "beliefs/obstacle_distribution.hpp"

namespace BTP
{
    class OmniscientStrategy : public Strategy
    {
    public:
        State &true_state;
        
        OmniscientStrategy(State &true_state, Location goal) :
            Strategy(*true_state.graph, goal), true_state(true_state)
        {
            name = "Omniscient";
        }

        virtual const std::string getBeliefName() const override
        {
            return "None";
        }
        

        virtual Action getNextAction(Location current, Observations obs) override;
    };

        
    class OptimisticStrategy : public Strategy
    {
    public:
        OptimisticStrategy(GraphD graph, Location goal) :
            Strategy(graph, goal)
        {
            name = "NaiveOptimistic";
        }

        virtual const std::string getBeliefName() const override
        {
            return "None";
        }

        
        void updateBelief(Observation obs)
        {
            using namespace arc_dijkstras;
            GraphEdge& e = graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID); 
        }
        
        virtual Action getNextAction(Location current, Observations obs) override;
    };


    class OptimisticWithPrior : public Strategy
    {
    public:
        std::unique_ptr<Belief> bel;
        int num_samples;
    public:
        OptimisticWithPrior(GraphD graph, Location goal, const Belief &bel):
            Strategy(graph, goal), bel(bel.clone()), num_samples(100)
        {
            name = "Optimistic";
        }

        virtual const std::string getBeliefName() const override
        {
            return bel->getName();
        }


        void updateEdges();

        virtual Action getNextAction(Location current, Observations obs) override;

        void viz(GraphVisualizer &viz) const override
        {
            bel->viz(viz);
        }

    };

    
    class AverageOverClairvoyance : public Strategy
    {
    public:
        std::unique_ptr<Belief> bel;
        int num_samples;
    public:
        AverageOverClairvoyance(GraphD graph, Location goal, const Belief &bel) :
            Strategy(graph, goal), bel(bel.clone()), num_samples(100)
        {
            name = "Averaging_over_clairvoyance";
        }

        virtual const std::string getBeliefName() const override
        {
            return bel->getName();
        }


        Action planPathInEnv(const State &s);

        virtual Action getNextAction(Location current, Observations obs) override;

        void viz(GraphVisualizer &viz) const override
        {
            bel->viz(viz);
        }
    };
}


#endif
