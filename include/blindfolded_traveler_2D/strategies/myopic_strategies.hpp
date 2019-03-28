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
        {}

        virtual Action getNextAction(Location current, Observations obs) override
        {
            if(current == goal)
            {
                return goal;
            }
            using namespace arc_dijkstras;

            const auto distance_fn = [&] (const Graph<std::vector<double>>& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    return edge.getWeight();
                };

            const auto edge_validity_check_fn = [&] (const Graph<std::vector<double>>& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    return true_state.getBlockage(edge.getFromIndex(), edge.getToIndex()) >= 1;
                };

            auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformLazyAstar(
                graph, current, goal,
                edge_validity_check_fn,
                distance_fn, 
                &distanceHeuristic, true);
            return result.first[1];
        }
    };

        
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
