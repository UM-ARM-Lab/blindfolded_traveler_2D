#ifndef BTP_STATE_HPP
#define BTP_STATE_HPP
#include "graph_planner/halton_graph.hpp"

namespace BTP
{
    typedef int64_t Location;
    typedef int64_t Action;

    /**
     *   Abstract State class
     */
    class State
    {
    public:
        GraphD graph;
        Location current_location;

    public:
        State(GraphD graph, Location cur) :
            graph(graph), current_location(cur)
        {};

        
        virtual double getBlockage(Location l, Action a) = 0;

        std::vector<Action> getActions(Location l)
        {
            std::vector<Action> neighbors;
            for(auto edge: graph.getNode(l).getOutEdges())
            {
                neighbors.push_back(edge.getToIndex());
            }
            return neighbors;
        }
    };



    class IndependentBlockageState : public State
    {
    public:
        IndependentBlockageState(GraphD graph, Location cur) :
            State(graph, cur)
        {};
        
        double getBlockage(Location l, Action a) override
        {
            return 1;
        }
    };
}


#endif
