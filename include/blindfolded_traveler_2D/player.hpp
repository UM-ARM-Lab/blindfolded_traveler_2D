#ifndef BTPPLAYER_HPP
#define BTPPLAYER_HPP

#include "scenarios/scenario.hpp"
#include "strategies/strategy.hpp"
#include "graph_planner/graph_visualization.hpp"
#include "arc_utilities/timing.hpp"


namespace BTP
{
    template <typename T>
    inline std::string Str(const T &t)
    {
        std::ostringstream os;
        os << t;
        return os.str();
    }
    
    class Player
    {
    public:
        ros::NodeHandle& n;
        GraphVisualizer viz;

        Player(ros::NodeHandle &n) : n(n), viz(n)
        {}

        void displayTitles(const Scenario &scenario, const Strategy &strat)
        {
            viz.vizText(Str("Scenario: ") + scenario.getName(), 1, 0.5, 1.1, "scenario");
            viz.vizText(Str("Strategy: ") + strat.getName(), 1, 0.5, 1.05, "strategy");
        }

        void reportStats(Scenario &scenario)
        {
            std::cout << "Total Cost: " << scenario.accumulated_cost << "\n";
            std::cout << "Edges Attempted: " << scenario.edges_attempted << "\n";
            std::cout << "Invalid Attempted: " << scenario.invalid_edges_attempted << "\n";
        }

        void recordStats(Scenario &scenario)
        {
            PROFILE_RECORD_DOUBLE("ExecutionCost", scenario.accumulated_cost);
            PROFILE_RECORD_DOUBLE("EdgesAttempted", scenario.edges_attempted);
            PROFILE_RECORD_DOUBLE("InvalidEdgesAttempted", scenario.invalid_edges_attempted);
        }
        
        bool run(Scenario &scenario, Strategy &strat, double sleep_time_s = 0)
        {
            scenario.viz(viz);

            displayTitles(scenario, strat);
            std::cout << "Agent starting at " << scenario.getLocation() << " with goal " << strat.goal << "\n";

            PROFILE_RECORD_DOUBLE("Strategy: " + strat.getName(), 0);
            PROFILE_RECORD_DOUBLE("Scenario: " + scenario.getName(), 0);

            int action_count = 0;
            int action_limit = 30;

            while(!scenario.completed() && action_count < action_limit)
            {
                PROFILE_START("Planning Action");
                Action a = strat.getNextAction(scenario.getLocation(), scenario.getObservations(), viz);
                PROFILE_RECORD("Planning Action");
                std::cout << "Agent at " << scenario.getLocation() << " taking action " << a << "\n";
                
                scenario.transition(a);
                strat.viz(viz);                
                
                ros::Duration(sleep_time_s).sleep();
                action_count++;
            }

            //run one final time to update belief for visualization
            strat.getNextAction(scenario.getLocation(), scenario.getObservations(), viz);

            reportStats(scenario);
            recordStats(scenario);


            if(action_count == action_limit)
            {
                std::cout << "Exiting without success because action limit reached\n";
                PROFILE_RECORD_DOUBLE("Action_Limit_Exceeded", action_count);
                return false;
            }
            return true;
        }
    };
}


#endif
