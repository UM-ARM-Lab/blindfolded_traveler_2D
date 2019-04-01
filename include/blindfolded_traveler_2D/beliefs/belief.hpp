#ifndef BTP_BELIEF_HPP
#define BTP_BELIEF_HPP
#include "states/state.hpp"
#include "observations.hpp"
#include <random>

namespace BTP
{
    typedef std::pair<std::shared_ptr<State>, double> WeightedState;
    
    class ExplicitBelief
    {
    public:
        virtual std::shared_ptr<State> sample(std::mt19937 &rng) const = 0;
        virtual std::vector<WeightedState> getWeightedStates() const = 0;
        virtual void update(Observation obs) = 0;
    };
}



#endif
