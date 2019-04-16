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

    inline void shift(Obstacles2D::Rect& r, double x, double y)
    {
        r.x1 += x;
        r.x2 += x;
        r.y1 += y;
        r.y2 += y;
    }

    inline void shiftToContactPoint(Obstacles2D::Rect *r, std::vector<double> collision_q,
                                    std::vector<double> from, std::vector<double> to)
    {
        double eps = 0.000001;
        double cx = collision_q[0];
        double cy = collision_q[1];

        //if edge goes left to right, collision occurred on obstacle left edge
        bool match_left_edge = from[0] < to[0];

        //if edge goes down to up, collision occurred on obstacle bottom
        bool match_bottom_edge = from[1] < to[1];


        double shiftx = cx - (match_left_edge ?   r->x1 + eps : r->x2 - eps);
        double shifty = cy - (match_bottom_edge ? r->y1 + eps : r->y2 - eps);


        if(from[1] == to[1]) //horizontal edge in graph
        {
            shift(*r, shiftx, 0); //make side wall of rect match collision point
            
            if(cy < r->y1) //shift up or down as needed
            {
                shift(*r, 0, cy - r->y1 - eps);
            }
            if(cy > r->y2)
            {
                shift(*r, 0, cy - r->y2 + eps);
            }
            return;
        }
        
        if(from[0] == to[0]) //pure vertical edge in graph
        {
            shift(*r, 0, shifty);

            if(cx < r->x1)
            {
                shift(*r, cx - r->x1 - eps, 0);
            }
            if(cx > r->x2)
            {
                shift(*r, cx - r->x2 + eps, 0);
            }
            return;
        }

        if(r->y1 < cy && cy < r->y2)
        {
            shift(*r, shiftx, 0);
            return;
        }
        else if(r->x1 < cx && cx < r->x2)
        {
            shift(*r, 0, shifty);
            return;
        }

        shift(*r, shiftx, shifty);

    }


    /*
     *  Shifts obstacles around to make them consistent with the provided observation
     *
     *   NOTE!!! Currently only works to shift one obstacle to be consistent with a collision observation
     *   This does not move obstacles out of the way of collision
     */
    inline void makeConsistentCollision(ObstacleState& s, Observation obs)
    {
        
        using namespace Obstacles2D;
        
        auto from = s.graph->getNode(obs.from).getValue();
        auto to = s.graph->getNode(obs.to).getValue();
        double eps = 0.000001;
        std::vector<double> collision_q = EigenHelpers::Interpolate(from, to, obs.blockage + eps);
        Rect* r = dynamic_cast<Rect*>(getNearest(s.obstacles, collision_q).get());
        shiftToContactPoint(r, collision_q, from, to);
    }



    inline void makeConsistentFree(ObstacleState& s, Observation obs)
    {
        using namespace Obstacles2D;
        for(auto obstacle: s.obstacles.obs)
        {
            Obstacles test_obstacle;
            test_obstacle.obs.push_back(obstacle);
                
            if(!test_obstacle.isValid(s.graph->getEdge(obs.from, obs.to), *s.graph))
            {
                // std::cout << "Moving obstacle to oblivion\n";
                Rect* r = dynamic_cast<Rect*>(obstacle.get());
                r->x1 = 1000;
                r->x2 = 1001;
                r->y1 = 1000;
                r->y2 = 1001;
            }
        }
    }

    inline void makeConsistent(ObstacleState& s, Observation obs)
    {
        if(obs.succeeded())
        {
            makeConsistentFree(s, obs);
        }
        else
        {
            makeConsistentCollision(s, obs);
        }
    }
}


#endif
