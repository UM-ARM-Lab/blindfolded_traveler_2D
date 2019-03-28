#ifndef BTP_SCENARIO_HPP
#define BTP_SCENARIO_HPP
#include "states/state.hpp"
#include "observations.hpp"
#include "graph_planner/graph_visualization.hpp"

namespace BTP
{
    inline bool isConsistent(const Observation& obs, const ObstacleState& s)
    {
        Observation should(obs.from, obs.to, s.getBlockage(obs.from, obs.to));
        return should.succeeded() == obs.succeeded();
    }
    

    class Scenario
    {
    public:
        Location goal;
        double accumulated_cost;
        int edges_attempted;
        int invalid_edges_attempted;
        Observations obs;

    public:
        Scenario() :
            goal(),
            accumulated_cost(0.0),
            edges_attempted(0),
            invalid_edges_attempted(0)
        {}
        
        Scenario(Location goal) :
            goal(goal),
            accumulated_cost(0.0),
            edges_attempted(0),
            invalid_edges_attempted(0)
        {}
        
        virtual bool completed() const
        {
            return getLocation() == goal;
        }

        virtual const GraphD& getGraph() const = 0;

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

    protected:
        virtual State& getState() = 0;

    };

}



#endif
