#ifndef BTP_STATE_HPP
#define BTP_STATE_HPP
#include "btp_defines.hpp"
#include "graph_planner/halton_graph.hpp"
#include "graph_planner/2d_obstacles.hpp"

namespace BTP
{
    /**
     *   Abstract State class
     */
    class State
    {
    public:
        const GraphD *graph;
        Location current_location;

    public:
        State(const GraphD *graph, Location cur) :
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

        
        bool pathExists(int goal) const
        {
            using namespace arc_dijkstras;

            const auto distance_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    return edge.getWeight();
                };

            const auto edge_validity_check_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    return getBlockage(edge.getFromIndex(), edge.getToIndex()) >= 1;
                };

            auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformLazyAstar(
                *graph, current_location, goal,
                edge_validity_check_fn,
                distance_fn, 
                &distanceHeuristic, true);
            return result.second < std::numeric_limits<double>::max();
        }
        
    };



    class IndependentBlockageState : public State
    {
    public:
        std::set<arc_dijkstras::HashableEdge> blocked_edges;
        
    public:
        IndependentBlockageState(const GraphD* graph, Location cur, std::set<arc_dijkstras::HashableEdge> blocked_edges) :
            State(graph, cur),
            blocked_edges(blocked_edges)
        {};

        virtual std::unique_ptr<State> clone() const override
        {
            return std::make_unique<IndependentBlockageState>(graph, current_location, blocked_edges);
        }

        double getBlockage(Location l, Action a) const override
        {
            arc_dijkstras::HashableEdge e(l, a);
            if(blocked_edges.count(e))
            {
                return 0.5; //TODO: This just blocks the edge half way. Not very sophistocated
            }
            return 1.0;
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
