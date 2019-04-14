#ifndef BTP_DEFINES_HPP
#define BTP_DEFINES_HPP
#include <cstdint>

namespace BTP
{
    typedef int64_t Location;
    typedef int64_t Action;

    static const double min_obstacle_length = 0.3;
    static const double robot_width = min_obstacle_length;
}

#endif
