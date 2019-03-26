#ifndef BTP_STATE_HPP
#define BTP_STATE_HPP
#include "graph_planner/halton_graph.hpp"
#include "graph_planner/2d_obstacles.hpp"

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
        {}

        State()
        {}

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
            return 1; //TODO define a blockage variable and return based on it
        }
    };


    class ObstacleState : public State
    {
    public:
        ObstacleState()
        {
        }
        
        ObstacleState(GraphD graph, Location cur, Obstacles2D::Obstacles obstacles):
            State(graph, cur), obstacles(obstacles)
        {
        }

        virtual double getBlockage(Location l, Action a) override
        {
            std::vector<double> q1 = graph.getNode(l).getValue();
            std::vector<double> q2 = graph.getNode(a).getValue();
            return obstacles.fractionUntilCollision(q1, q2);
        }
        
        Obstacles2D::Obstacles obstacles;
    };
}


#endif
