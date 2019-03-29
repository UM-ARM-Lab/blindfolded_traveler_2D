#ifndef BTP_STRATEGY_HPP
#define BTP_STRATEGY_HPP
#include "graph_planner/halton_graph.hpp"
#include "states/state.hpp"
#include "observations.hpp"
#include "graph_planner/graph_visualization.hpp"

namespace BTP
{
    class Strategy
    {
    public:
        GraphD graph;
        Location goal;

    protected:
        std::string name;
        
    public:
        Strategy(GraphD graph, Location goal) :
            graph(graph), goal(goal), name("unset")
        {}

        virtual Action getNextAction(Location current, Observations obs) = 0;

        virtual Action getNextAction(Location current, Observations obs, GraphVisualizer &viz)
        {
            Action a = getNextAction(current, obs);
            viz.vizGraph(graph, "Strategy Belief");
            viz.vizPoints(std::vector<Location>{current, goal}, graph);
            viz.vizPath(std::vector<Location>{current, a}, graph);
            return a;
        }

        virtual void viz(GraphVisualizer &viz) const
        {
        }

        virtual std::string getName() const
        {
            return name;
        }
    };
}


#endif
