#ifndef BTP_STRATEGY_HPP
#define BTP_STRATEGY_HPP
#include "graph_planner/halton_graph.hpp"
#include "states/state.hpp"
#include "observations.hpp"

namespace BTP
{
    class Strategy
    {
    public:
        GraphD graph;
        Location goal;
        
    public:
        Strategy(GraphD graph, Location goal) :
            graph(graph), goal(goal)
        {}

        virtual Action getNextAction(Location current, Observations obs) = 0;
    };
}


#endif
