#ifndef BTP_OBSTACLE_PROJECTION_HPP
#define BTP_OBSTACLE_PROJECTION_HPP

#include "graph_planner/2d_obstacles.hpp"
#include "observations.hpp"
#include "arc_utilities/eigen_helpers.hpp"
#include "states/state.hpp"

namespace BTP
{
    inline std::shared_ptr<Obstacles2D::Obstacle> getNearest(Obstacles2D::Obstacles o,
                                                             const std::vector<double>& q)
    {
        double smallest_distance = std::numeric_limits<double>::max();
        int smallest_index = -1;
        
        for(int i=0; i<o.obs.size(); i++)
        {
            double d = o.obs[i]->distance(q);
            if(d < smallest_distance)
            {
                smallest_distance = d;
                smallest_index = i;
            }
        }
        return o.obs[smallest_index];
    }

    inline void makeConsistent(ObstacleState& s, Observation obs)
    {
        using namespace Obstacles2D;
        
        auto from = s.graph->getNode(obs.from).getValue();
        auto to = s.graph->getNode(obs.to).getValue();
        double eps = 0.000001;
        std::vector<double> collision_q = EigenHelpers::Interpolate(from, to, obs.blockage + eps);
        

        Rect* r = dynamic_cast<Rect*>(getNearest(s.obstacles, collision_q).get());

        //if edge goes left to right, collision occurred on obstacle left edge
        bool match_left_edge = from[0] < to[0];

        //if edge goes down to up, collision occurred on obstacle bottom
        bool match_bottom_edge = from[1] < to[1];

        // r->distance(
        // std::cout << "collision point is " << collision_q[0] << ", " << collision_q[1] << "\n";
        
        if(collision_q[1] > r->y1 && collision_q[1] < r->y2)
        {
            if(match_left_edge)
            {
                r->x2 += collision_q[0] - r->x1;
                r->x1 = collision_q[0];
                // std::cout<< "Adjusted rectangle to " << r->x1 << ", " << r->x2 << "\n";
            }
        }
    }
}


#endif
