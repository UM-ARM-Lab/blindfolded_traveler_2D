#ifndef BTP_SCENARIO_HPP
#define BTP_SCENARIO_HPP
#include "states/state.hpp"
#include "observations.hpp"
#include "graph_planner/graph_visualization.hpp"

namespace BTP
{
    

    class Scenario
    {
    public:
        GraphD graph;
        Location goal;
        double accumulated_cost;
        int edges_attempted;
        int invalid_edges_attempted;
        Observations obs;


    protected:
        std::string name;

    public:
        // Scenario() :
        //     goal(),
        //     accumulated_cost(0.0),
        //     edges_attempted(0),
        //     invalid_edges_attempted(0),
        //     name("Unset name")
        // {}
        
        Scenario(GraphD graph, Location goal) :
            graph(graph),
            goal(goal),
            accumulated_cost(0.0),
            edges_attempted(0),
            invalid_edges_attempted(0),
            name("Unset name")
        {}

        virtual std::string getName() const
        {
            return name;
        }
        
        virtual bool completed() const
        {
            return getLocation() == goal;
        }

        virtual const GraphD& getGraph() const
        {
            return graph;
        }

        virtual const Location& getLocation() const = 0;

        virtual const Observations& getObservations() const = 0;

        virtual void transition(Action a)
        {
            edges_attempted++;
            
            Location cur = getLocation();
            double b = getState().getBlockage(cur, a);
            double weight = getGraph().getEdge(cur, a).getWeight();
            Observation ob(cur, a, b);
            obs.push_back(ob);

            if(ob.succeeded())
            {
                getState().current_location = a;
                accumulated_cost += weight;
            }
            else
            {
                invalid_edges_attempted++;
                accumulated_cost += 2 * b * weight;
            }
        }

        virtual void viz(GraphVisualizer &viz) const 
        {
        }

        
    public:  //Public for testing omniscient strategies
        virtual State& getState() = 0;

    };

}



#endif
