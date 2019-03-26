#ifndef BTP_OBSTACLE_SCENARIO_HPP
#define BTP_OBSTACLE_SCENARIO_HPP
#include "scenarios/scenario.hpp"
#include "graph_planner/halton_graph.hpp"


namespace BTP
{
    class ObstacleScenario : public Scenario
    {
    public:
        ObstacleScenario()
        {
        }        
        
        ObstacleScenario(ObstacleState s, Location goal):
            Scenario(goal),
            true_state(s)
        {
        }
        

        virtual const GraphD& getGraph() const override
        {
            return true_state.graph;
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
        }



    protected:
        ObstacleState true_state;

        virtual State& getState()
        {
            return true_state;
        }
    };





    class WallObstacleScenario : public ObstacleScenario
    {
    public:
        WallObstacleScenario()
        {
            int num_rows = 5;
            Grid grid(5);
            int start = 0;
            goal = 24;
            
            Obstacles2D::Obstacles o;
            o.obs.push_back(std::make_shared<Obstacles2D::Rect>(0.4, -0.1, 0.6, 0.95));
            true_state = ObstacleState(ObstacleState(grid, start, o));
        }
    };
}

#endif
