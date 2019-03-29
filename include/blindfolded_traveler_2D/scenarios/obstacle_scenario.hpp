#ifndef BTP_OBSTACLE_SCENARIO_HPP
#define BTP_OBSTACLE_SCENARIO_HPP
#include "scenarios/scenario.hpp"
#include "graph_planner/halton_graph.hpp"


namespace BTP
{
    class ObstacleScenario : public Scenario
    {
    public:

        /**
         *  Construct a scenario with no initial obstacles. They can be populated later
         */
        ObstacleScenario(const GraphD& g, Location start, Location goal) :
            Scenario(g, goal),
            true_state(getGraph(), start)
        {
        }        
        
        ObstacleScenario(ObstacleState s, Location goal):
            Scenario(s.graph, goal),
            true_state(s)
        {
        }

        virtual const Location& getLocation() const override
        {
            return true_state.current_location;
        }

        virtual const Observations& getObservations() const override
        {
            return obs;
        }
        
        virtual void viz(GraphVisualizer &viz) const override
        {
            viz.vizObstacles(true_state.obstacles);
            viz.vizObstacles(true_state.obstacles, 0.01, "True Obstacles");
        }



    protected:
        ObstacleState true_state;

        virtual State& getState()
        {
            return true_state;
        }
    };
}

#endif
