#ifndef BTP_MYOPIC_STRATEGY_HPP
#define BTP_MYOPIC_STRATEGY_HPP

#include "strategies/strategy.hpp"
#include "graph_planner/a_star.hpp"
#include "arc_utilities/eigen_helpers.hpp"
#include "distributions/obstacle_distribution.hpp"

namespace BTP
{
    class OmniscientStrategy : public Strategy
    {
    public:
        State &true_state;
        
        OmniscientStrategy(State &true_state, Location goal) :
            Strategy(true_state.graph, goal), true_state(true_state)
        {
            name = "Omniscient";
        }

        virtual Action getNextAction(Location current, Observations obs) override;
    };

        
    class OptimisticStrategy : public Strategy
    {
    public:
        OptimisticStrategy(GraphD graph, Location goal) :
            Strategy(graph, goal)
        {
            name = "Optimistic";
        }
        
        void updateBelief(Observation obs)
        {
            using namespace arc_dijkstras;
            GraphEdge& e = graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID); 
        }
        
        virtual Action getNextAction(Location current, Observations obs) override;
    };

    class BestExpectedStrategy : public Strategy
    {
    public:
        ObstacleDistribution obstacle_distribution;
        std::vector<bool> invalidated_belief;

    public:
        BestExpectedStrategy(GraphD graph, Location goal, ObstacleDistribution d) :
            Strategy(graph, goal), obstacle_distribution(d), invalidated_belief(obstacle_distribution.o.size())
        {
            name = "Best in expectation";
        }

        void markInvalidEnvironments(Observation obs);

        void updateBelief(Observation obs)
        {
            using namespace arc_dijkstras;
            GraphEdge& e = graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID);
            markInvalidEnvironments(obs);
        }

        Action planPathInEnv(const State &s);

        virtual Action getNextAction(Location current, Observations obs) override;
    };
}


#endif
