#ifndef BTP_BELIEF_HPP
#define BTP_BELIEF_HPP
#include "states/state.hpp"
#include "observations.hpp"
#include <random>
#include "graph_planner/graph_visualization.hpp"

namespace BTP
{
    typedef std::pair<std::shared_ptr<State>, double> WeightedState;


    class Belief
    {
    public:
        virtual std::unique_ptr<State> sample(std::mt19937 &rng) const = 0;
        virtual double getProbability(Observation obs) const = 0;
        virtual std::unique_ptr<Belief> clone() const = 0;
        virtual void update(Observation obs) = 0;
        virtual void viz(GraphVisualizer &viz) const = 0;
    };
    
    
    class ExplicitBelief : public Belief
    {
    public:
        virtual std::vector<WeightedState> getWeightedStates() const = 0;
        virtual std::unique_ptr<ExplicitBelief> cloneExplicit() const = 0;
    };
}



#endif
