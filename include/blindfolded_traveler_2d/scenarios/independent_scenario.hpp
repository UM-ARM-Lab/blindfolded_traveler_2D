#ifndef BTP_INDEPENDENT_SCENARIO_HPP
#define BTP_INDEPENDENT_SCENARIO_HPP
#include "scenarios/scenario.hpp"
#include "graph_planner/halton_graph.hpp"

//NOTE: This is a stub, unfinished


namespace BTP
{
    class IndependentBlockageScenario : public Scenario
    {
    public:
        
        IndependentBlockageScenario(const GraphD& g, Location start, Location goal) :
            Scenario(g, goal),
            true_state(&getGraph(), start)
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


    public: 
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
