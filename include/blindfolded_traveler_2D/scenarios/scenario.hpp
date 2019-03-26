#ifndef BTP_SCENARIO_HPP
#define BTP_SCENARIO_HPP
#include "states/state.hpp"
#include "observations.hpp"

namespace BTP
{

    class Scenario
    {
    public:
        Location goal;
        double accumulated_cost;
        Observations obs;

    public:
        Scenario(Location goal) :
            accumulated_cost(0), goal(goal)
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
                accumulated_cost += 2 * b * weight;
            }
        }

    protected:
        virtual State& getState() = 0;

    };

}



#endif
