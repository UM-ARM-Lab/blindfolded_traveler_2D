#ifndef BTPPLAYER_HPP
#define BTPPLAYER_HPP

#include "scenarios/scenario.hpp"
#include "strategies/strategy.hpp"
#include "graph_planner/graph_visualization.hpp"


namespace BTP
{
    class Player
    {
    public:
        ros::NodeHandle& n;
        GraphVisualizer viz;

        Player(ros::NodeHandle &n) : n(n), viz(n)
        {}
        
        void run(Scenario &scenario, Strategy &strat, double sleep_time_s = 0)
        {
            scenario.viz(viz);
            std::cout << "Agent starting at " << scenario.getLocation() << " with goal " << strat.goal << "\n";
            
            while(!scenario.completed())
            {
                Action a = strat.getNextAction(scenario.getLocation(), scenario.getObservations(), viz);
                std::cout << "Agent at " << scenario.getLocation() << " taking action " << a << "\n";
                
                scenario.transition(a);
                
                
                ros::Duration(sleep_time_s).sleep();
            }

            //run one final time to update belief for visualization
            strat.getNextAction(scenario.getLocation(), scenario.getObservations(), viz);
        }
    };
}


#endif
