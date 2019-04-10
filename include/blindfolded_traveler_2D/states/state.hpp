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
        const GraphD* graph;
        Location current_location;

    public:
        State(const GraphD* graph, Location cur) :
            graph(graph), current_location(cur)
        {}

        virtual std::unique_ptr<State> clone() const = 0;

        /*
         *  Returns the fraction (0.0 to 1.0) of the edge traversed before a collision was encountered
         *   
         */
        virtual double getBlockage(Location l, Action a) const = 0;

        std::vector<Action> getActions(Location l)
        {
            std::vector<Action> neighbors;
            for(auto edge: graph->getNode(l).getOutEdges())
            {
                neighbors.push_back(edge.getToIndex());
            }
            return neighbors;
        }

        virtual void debug() const = 0;
    };



    class IndependentBlockageState : public State
    {
    public:
        IndependentBlockageState(const GraphD* graph, Location cur) :
            State(graph, cur)
        {};

        virtual std::unique_ptr<State> clone() const override
        {
            return std::make_unique<IndependentBlockageState>(graph, current_location);
        }
        
        double getBlockage(Location l, Action a) const override
        {
            return 1; //TODO define a blockage variable and return based on it
        }

        virtual void debug() const override
        {}
    };


    class ObstacleState : public State
    {
    public:
        Obstacles2D::Obstacles obstacles;

    public:
        ObstacleState(const GraphD* graph, Location cur):
            State(graph, cur)
        {
        }
        
        ObstacleState(const GraphD* graph, Location cur, Obstacles2D::Obstacles obstacles):
            State(graph, cur), obstacles(obstacles)
        {
        }

        virtual std::unique_ptr<State> clone() const override
        {
            return std::make_unique<ObstacleState>(graph, current_location, obstacles);
        }


        virtual double getBlockage(Location l, Action a) const override
        {
            std::vector<double> q1 = graph->getNode(l).getValue();
            std::vector<double> q2 = graph->getNode(a).getValue();
            return obstacles.fractionUntilCollision(q1, q2);
        }
        

        virtual void debug() const override
        {
            
            auto& r = dynamic_cast<Obstacles2D::Rect&>(*obstacles.obs[0]);
            std::cout << r.x1 << ", " << r.y1 << ", " << r.x2 << ", " << r.y2 << "\n";
        }

    };
}


#endif
