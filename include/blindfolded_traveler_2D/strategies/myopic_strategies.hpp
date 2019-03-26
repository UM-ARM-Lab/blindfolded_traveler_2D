#ifndef BTP_MYOPIC_STRATEGY_HPP
#define BTP_MYOPIC_STRATEGY_HPP

#include "strategies/strategy.hpp"
#include "graph_planner/a_star.hpp"
#include "arc_utilities/eigen_helpers.hpp"

namespace BTP
{
    class OptimisticStrategy : public Strategy
    {
    public:
        OptimisticStrategy(GraphD graph, Location goal) :
            Strategy(graph, goal)
        {}
        
        void updateBelief(Observation obs)
        {
            using namespace arc_dijkstras;
            GraphEdge& e = graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID); 
        }
        
        virtual Action getNextAction(Location current, Observations obs) override
        {
            if(obs.size() > 0)
            {
                updateBelief(obs.back());
            }

            if(current == goal)
            {
                return goal;
            }

            auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformAstar(
                graph, current, goal, &distanceHeuristic, true);

            return result.first[1];
        }
    };
}


#endif
