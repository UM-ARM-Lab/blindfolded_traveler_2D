#include "strategies/myopic_strategies.hpp"
#include "scenarios/scenario.hpp"

using namespace BTP;



/******************************
 **   Omniscient Strategy
 ******************************/
Action OmniscientStrategy::getNextAction(Location current, Observations obs)
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




/******************************
 **   Optimistic Strategy
 ******************************/
Action OptimisticStrategy::getNextAction(Location current, Observations obs)
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


/******************************
 **   BestExpectedStrategy
 ******************************/
Action BestExpectedStrategy::planPathInEnv(const State &s)
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
            
    return result.first[1];
}


Action BestExpectedStrategy::getNextAction(Location current, Observations obs)
{
    if(obs.size() > 0)
    {
        bel.update(obs.back());
    }
    bel.setLocation(current);

    if(current == goal)
    {
        return goal;
    }

    std::map<Location, double> actions;
    using pair_type = decltype(actions)::value_type;
    for(WeightedState &ws: bel.getWeightedStates())
    {
        if(ws.second == 0)
        {
            continue;
        }
                
        Action a = planPathInEnv(*ws.first);

        if(actions.count(a) == 0)
        {
            actions[a] = 0;
        }
        actions[a] += ws.second;
    }

    auto pr = std::max_element(actions.begin(), actions.end(),
                               [](const pair_type &p1, const pair_type &p2)
                               {return p1.second < p2.second;});
    return pr->first;
}


