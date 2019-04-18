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

    if(result.second >= std::numeric_limits<double>::max())
    {
        throw std::logic_error("No valid path to goal");
    }

    return result.first[1];
}



/******************************
 **   OptimisticWithPrior Strategy
 ******************************/



void OptimisticWithPrior::updateEdges()
{
    std::mt19937 rng;
    rng.seed(0);

    std::vector<std::unique_ptr<State>> sampled_states;

    for(int i=0; i<num_samples; i++)
    {
        sampled_states.push_back(bel->sample(rng));
    }
    
    for(auto &n: graph.getNodes())
    {
        for(auto &e: n.getOutEdges())
        {
            if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::INVALID)
            {
                continue;
            }

            bool exists_valid = false;
            for(const auto& s: sampled_states)
            {
                if(s->getBlockage(e.getFromIndex(), e.getToIndex()) == 1)
                {
                    exists_valid = true;
                    break;
                }
            }

            if(!exists_valid)
            {
                e.setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
            }
        }
    }
}

Action OptimisticWithPrior::getNextAction(Location current, Observations obs)
{
    if(obs.size() > 0)
    {
        bel->update(obs.back());
    }
    updateEdges();

    if(current == goal)
    {
        return goal;
    }

    auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformAstar(
        graph, current, goal, &distanceHeuristic, true);

    if(result.second >= std::numeric_limits<double>::max())
    {
        throw std::logic_error("No valid path to goal");
    }

    return result.first[1];
}


/******************************
 **   AverageOverClairvoyance
 ******************************/
Action AverageOverClairvoyance::planPathInEnv(const State &s)
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

    if(result.first.size() < 2 || result.second >= std::numeric_limits<double>::max())
    {
        throw std::out_of_range("No Path Found");
    }
            
    return result.first[1];
}


Action AverageOverClairvoyance::getNextAction(Location current, Observations obs)
{
    if(obs.size() > 0)
    {
        bel->update(obs.back());
    }

    if(current == goal)
    {
        return goal;
    }

    std::map<Action, double> actions;
    using pair_type = decltype(actions)::value_type;

    std::mt19937 rng;
    // rng.seed(time(0));
    rng.seed(1337); //Consistent seed, to avoid alternating actions back an forth due to sample variance

    for(int i=0; i<num_samples; i++)
    {
        Action a;
        try
        {
            a = planPathInEnv(*bel->sample(rng));
        }
        catch (const std::out_of_range& e)
        {
            std::cout << "No path found, skipping this world\n";
            continue;
        }

        if(actions.count(a) == 0)
        {
            actions[a] = 0;
        }
        // std::cout << "Action " << a << " optimal for sampled world\n";
        actions[a] += 1.0;
    }

    // std::cout << "Action count\n";
    // for(const auto& kv: actions)
    // {
    //     std::cout << kv.first << ": " << kv.second << "\n";
    // }

    auto pr = std::max_element(actions.begin(), actions.end(),
                               [](const pair_type &p1, const pair_type &p2)
                               {return p1.second < p2.second;});
    return pr->first;
}


