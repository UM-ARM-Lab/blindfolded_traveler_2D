#ifndef BTPPLAYER_HPP
#define BTPPLAYER_HPP

#include "scenarios/scenario.hpp"
#include "strategies/strategy.hpp"



namespace BTP
{
    class Player
    {
    public:
        void run(Scenario &scenario, Strategy &strat)
        {
            while(!scenario.completed())
            {
                Action a = strat.getNextAction(scenario.getLocation(), scenario.getObservations());
                std::cout << "Agent at " << scenario.getLocation() << " taking action " << a << "\n";
                scenario.transition(a);
                std::cout << "Completed? " << scenario.completed() << "\n";
                std::cout << "Agent at " << scenario.getLocation() << " with goal " << scenario.goal << "\n";
            }
        }
    };
}


#endif
