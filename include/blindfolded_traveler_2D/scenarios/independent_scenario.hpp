#ifndef BTP_INDEPENDENT_SCENARIO_HPP
#define BTP_INDEPENDENT_SCENARIO_HPP
#include "scenarios/scenario.hpp"
#include "graph_planner/halton_graph.hpp"


namespace BTP
{
    class IndependentBlockageScenario : public Scenario
    {
    public:
        
        // IndependentBlockageScenario(IndependentBlockageState s, Location goal):
        //     Scenario(s.graph, goal),
        //     true_state(s)
        // {
        // }
        IndependentBlockageScenario(const GraphD& g, Location start, Location goal) :
            Scenario(g, goal),
            true_state(getGraph(), start)
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


    protected:
        IndependentBlockageState true_state;

        virtual State& getState()
        {
            return true_state;
        }
    };





    class IndependentBlockageGridScenario : public IndependentBlockageScenario
    {
    public:
        IndependentBlockageGridScenario(int num_rows):
            IndependentBlockageScenario(Grid(num_rows), 0, num_rows * num_rows - 1)
        {
        }
    };
}

#endif
