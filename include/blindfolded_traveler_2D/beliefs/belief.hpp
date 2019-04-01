#ifndef BTP_BELIEF_HPP
#define BTP_BELIEF_HPP
#include "states/state.hpp"
#include "observations.hpp"
#include <random>

namespace BTP
{
    // typedef std::pair<S
    
    class ExplicitBelief
    {
    public:
        virtual std::shared_ptr<State> sample(std::mt19937 &rng) const = 0;
        // getWeightedStates
        // virtual void update(
    };
}



#endif
