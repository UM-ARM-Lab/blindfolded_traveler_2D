#ifndef BTP_OBSERVATIONS_HPP
#define BTP_OBSERVATIONS_HPP
#include <vector>

namespace BTP
{
    class Observation
    {
    public:
        Location from;
        Action to;
        double blockage;
    public:
        Observation(Location from, Location to, double blockage) :
            from(from), to(to), blockage(blockage) {}
        bool succeeded() { return blockage >= 1;}
    };

    typedef std::vector<Observation> Observations;
}


#endif
