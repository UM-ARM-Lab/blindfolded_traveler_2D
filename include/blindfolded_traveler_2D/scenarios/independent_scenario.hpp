#ifndef BTP_INDEPENDENT_SCENARIO_HPP
#define BTP_INDEPENDENT_SCENARIO_HPP
#include "scenarios/scenario.hpp"
#include "graph_planner/halton_graph.hpp"


namespace BTP
{
    class IndependentBlockageScenario : public Scenario
    {
    public:
        IndependentBlockageScenario(IndependentBlockageState s, Location goal):
            Scenario(goal),
            true_state(s)
        {
        }
        
        virtual void transition(Action a) override
        {
            Location cur = getLocation();
            double b = true_state.getBlockage(cur, a);
            double weight = getGraph().getEdge(cur, a).getWeight();
            Observation ob(cur, a, b);
            obs.push_back(ob);

            if(ob.succeeded())
            {
                true_state.current_location = a;
                accumulated_cost += weight;
            }
            else
            {
                accumulated_cost += 2 * b * weight;
            }
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


    protected:
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
            IndependentBlockageScenario(IndependentBlockageState(Grid(num_rows), 0), num_rows * num_rows - 1)
        {
        }
    };
}

#endif
