#ifndef BTP_SCENARIO_HPP
#define BTP_SCENARIO_HPP
#include "states/state.hpp"

namespace BTP
{

    class Scenario
    {
    public:
        Location goal;
        double accumulated_cost;

    public:
        Scenario() : accumulated_cost(0)
        {}
        
        virtual bool completed() const
        {
            return getLocation() == goal;
        }

        virtual const GraphD& getGraph() const = 0;

        virtual const Location getLocation() const = 0;

        virtual void Transition(Action a) = 0;

    };

}



#endif
