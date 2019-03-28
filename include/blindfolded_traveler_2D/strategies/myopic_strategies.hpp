#ifndef BTP_MYOPIC_STRATEGY_HPP
#define BTP_MYOPIC_STRATEGY_HPP

#include "strategies/strategy.hpp"
#include "graph_planner/a_star.hpp"
#include "arc_utilities/eigen_helpers.hpp"
#include "distributions/obstacle_distribution.hpp"

namespace BTP
{
    class OmniscientStrategy : public Strategy
    {
    public:
        State &true_state;
        
        OmniscientStrategy(State &true_state, Location goal) :
            Strategy(true_state.graph, goal), true_state(true_state)
        {}

        virtual Action getNextAction(Location current, Observations obs) override
        {
            if(current == goal)
            {
                return goal;
            }
            using namespace arc_dijkstras;

            const auto distance_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    return edge.getWeight();
                };

            const auto edge_validity_check_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    return true_state.getBlockage(edge.getFromIndex(), edge.getToIndex()) >= 1;
                };

            auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformLazyAstar(
                graph, current, goal,
                edge_validity_check_fn,
                distance_fn, 
                &distanceHeuristic, true);
            return result.first[1];
        }
    };

        
    class OptimisticStrategy : public Strategy
    {
    public:
        OptimisticStrategy(GraphD graph, Location goal) :
            Strategy(graph, goal)
        {}
        
        void updateBelief(Observation obs)
        {
            using namespace arc_dijkstras;
            GraphEdge& e = graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID); 
        }
        
        virtual Action getNextAction(Location current, Observations obs) override
        {
            if(obs.size() > 0)
            {
                updateBelief(obs.back());
            }

            if(current == goal)
            {
                return goal;
            }

            auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformAstar(
                graph, current, goal, &distanceHeuristic, true);

            return result.first[1];
        }
    };

    class BestExpectedStrategy : public Strategy
    {
    public:
        ObstacleDistribution obstacle_distribution;
        std::vector<bool> invalidated_belief;

    public:
        BestExpectedStrategy(GraphD graph, Location goal, ObstacleDistribution d) :
            Strategy(graph, goal), obstacle_distribution(d), invalidated_belief(obstacle_distribution.o.size())
        {}

        void markInvalidEnvironments(Observation obs)
        {
            for(int i=0; i<obstacle_distribution.o.size(); i++)
            {
                if(invalidated_belief[i])
                {
                    continue;
                }
                ObstacleState h(graph, obs.from, obstacle_distribution.o[i]);
                invalidated_belief[i] = !isConsistent(obs, h);
            }

            double count_valid = 0;
            for(auto inv: invalidated_belief)
            {
                count_valid += !inv;
            }
            std::cout << count_valid << " valid possible obstacle configurations\n";
        }

        void updateBelief(Observation obs)
        {
            using namespace arc_dijkstras;
            GraphEdge& e = graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID);
            markInvalidEnvironments(obs);
        }

        Action planPathInEnv(const State &s)
        {
            using namespace arc_dijkstras;

            const auto distance_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    return edge.getWeight();
                };

            const auto edge_validity_check_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    bool valid = s.getBlockage(edge.getFromIndex(), edge.getToIndex()) >= 1;
                    // std::cout << "Edge check: " << edge.getFromIndex() << ", " << edge.getToIndex();
                    // std::cout << ": " << valid << "\n";
                    return valid;
                };

            auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformLazyAstar(
                graph, s.current_location, goal,
                edge_validity_check_fn,
                distance_fn, 
                &distanceHeuristic, true);
            // std::cout << "Path found with cost " << result.second << "\n";

            // if(result.second > 100000)
            // {
            //     s.debug();
            //     ros::NodeHandle n;
            //     GraphVisualizer viz(n);
            //     auto &os = dynamic_cast<const ObstacleState&>(s);
            //     ros::Duration(1.0).sleep();
            //     viz.vizObstacles(os.obstacles);
            //     ros::Duration(1.0).sleep();
            // }
            
            return result.first[1];
        }

        


        virtual Action getNextAction(Location current, Observations obs) override
        {
            if(obs.size() > 0)
            {
                updateBelief(obs.back());
            }

            if(current == goal)
            {
                return goal;
            }

            std::map<Location, double> actions;
            using pair_type = decltype(actions)::value_type;
            for(int i=0; i<obstacle_distribution.o.size(); i++)
            {
                if(invalidated_belief[i])
                {
                    continue;
                }
                
                const auto &obstacles = obstacle_distribution.o[i];
                Action a = planPathInEnv(ObstacleState(graph, current, obstacles));

                if(actions.count(a) == 0)
                {
                    actions[a] = 0;
                }
                actions[a] += obstacle_distribution.weights[i];
            }

            auto pr = std::max_element(actions.begin(), actions.end(),
                                       [](const pair_type &p1, const pair_type &p2)
                                       {return p1.second < p2.second;});
            return pr->first;
        }
    };
}


#endif
