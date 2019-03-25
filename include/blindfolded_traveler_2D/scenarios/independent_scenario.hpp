#ifndef BTP_INDEPENDENT_SCENARIO_HPP
#define BTP_INDEPENDENT_SCENARIO_HPP
#include "scenarios/scenario.hpp"
#include "graph_planner/halton_graph.hpp"


namespace BTP
{
    class IndependentBlockageScenario : public Scenario
    {
    public:
        IndependentBlockageScenario(IndependentBlockageState s):
            true_state(s)
        {
        }
        
        virtual void Transition(Action a) override
        {
            double b = true_state.getBlockage(getLocation(), a);
            double weight = getGraph().getEdge(getLocation(), a).getWeight();
        }

        virtual const GraphD& getGraph() const override
        {
            return true_state.graph;
        }

        virtual const Location getLocation() const override
        {
            return true_state.current_location;
        }

    private:
        IndependentBlockageState true_state;
    };


    
    class Grid : public RDiscGraph
    {
    public:
        Grid(int rows):
            RDiscGraph(1.0/((double)rows - 1.0000001) * 1.4143)
        {
            for(int i=0; i<rows; i++)
            {
                for(int j=0; j<rows; j++)
                {
                    std::vector<double> q{(double)i / ((double)rows - 1.0),
                            (double)j / ((double)rows - 1.0)};
                    addVertexAndEdges(q);
                }
            }
        }
    };



    class IndependentBlockageGridScenario : public IndependentBlockageScenario
    {
    public:
        IndependentBlockageGridScenario(int num_rows):
            IndependentBlockageScenario(IndependentBlockageState(Grid(num_rows), 0))
        {
        }
    };
}

#endif
