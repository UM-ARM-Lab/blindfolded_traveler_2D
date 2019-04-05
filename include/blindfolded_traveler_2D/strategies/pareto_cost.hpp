#ifndef BTP_PARETO_COST_HPP
#define BTP_PARETO_COST_HPP

#include "strategies/strategy.hpp"
#include "beliefs/obstacle_distribution.hpp"

namespace BTP
{
    class ParetoCost : public Strategy
    {
    public:
        double collision_probability_weight;
        std::unique_ptr<Belief> bel;

    public:
        ParetoCost(GraphD graph, Location goal, const Belief &bel, double col_weight) :
            Strategy(graph, goal), bel(bel.clone()), collision_probability_weight(col_weight)
        {
            name = "ParetoCost";
        }

        virtual Action getNextAction(Location current, Observations obs) override
        {
            if(obs.size() > 0)
            {
                bel->update(obs.back());
                updateGraph(obs.back());
            }

            if(current == goal)
            {
                return goal;
            }
            return getParetoAction(current);
        }

        void viz(GraphVisualizer &viz) const override
        {
            bel->viz(viz);
        }

    protected:
        Action getParetoAction(Location current)
        {
            using namespace arc_dijkstras;
            
            const auto distance_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    Observation free_obs(edge.getFromIndex(), edge.getToIndex(), 1.0);
                    double log_prob =  - std::log(bel->getLikelihood(free_obs));
                    return edge.getWeight() + collision_probability_weight * log_prob;
                };

            const auto edge_validity_check_fn = [&] (const GraphD& search_graph, const GraphEdge& edge)
                {
                    UNUSED(search_graph);
                    if(edge.getValidity() == EDGE_VALIDITY::INVALID)
                    {
                        return false;
                    }

                    return edge.getWeight() < std::numeric_limits<double>::infinity();
                };
            
            auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformLazyAstar(
                graph, current, goal,
                edge_validity_check_fn,
                distance_fn,
                &distanceHeuristic, true);
            return result.first[1];
        }

        void updateGraph(Observation obs)
        {
            using namespace arc_dijkstras;
            GraphEdge& e = graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID); 
        }

    };
}


#endif
