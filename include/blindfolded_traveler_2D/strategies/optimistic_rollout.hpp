#ifndef BTP_MCTS_HPP
#define BTP_MCTS_HPP

#include "strategies/strategy.hpp"
#include "beliefs/obstacle_distribution.hpp"

namespace BTP
{
    class OptimisticRollout : public Strategy
    {
    public:
        int num_rollouts;
        ObstacleBelief bel;
        
    public:
        OptimisticRollout(GraphD graph, Location goal, ObstacleBelief bel) :
            Strategy(graph, goal), bel(bel), num_rollouts(100)
        {
            name = "Optimistic Rollout";
        }




        virtual Action getNextAction(Location current, Observations obs) override
        {
            if(obs.size() > 0)
            {
                bel.update(obs.back());
                updateGraph(graph, obs.back());
            }
            bel.setLocation(current);

            if(current == goal)
            {
                return goal;
            }

            std::mt19937 rng;
            rng.seed(time(0));

            std::map<Action, double> actions;
            
            for(int i=0; i<num_rollouts; i++)
            {
                std::shared_ptr<State> sampled_state = bel.sample(rng);
                auto possible_actions = sampled_state->getActions(sampled_state->current_location);

                for(auto initial_action: possible_actions)
                {
                    std::shared_ptr<State> rollout_state = sampled_state->clone();
                    GraphD rollout_graph = graph;
                    double rollout_cost = 0;
                    Observation ob = transition(rollout_state.get(), initial_action, rollout_cost);
                    updateGraph(rollout_graph, ob);

                    rollout_cost += rolloutOptimistic(rollout_graph, rollout_state.get());

                    if(actions.count(initial_action) == 0)
                    {
                        actions[initial_action] = 0;
                    }
                    actions[initial_action] += rollout_cost;
                }
            }

            using pair_type = decltype(actions)::value_type;
            auto pr = std::min_element(actions.begin(), actions.end(),
                                       [](const pair_type &p1, const pair_type &p2)
                                       {return p1.second < p2.second;});
            return pr->first;
        }

        void viz(GraphVisualizer &viz) const override
        {
            bel.viz(viz);
        }


    protected:
        Action optimisticPath(GraphD& rollout_graph, const State *s)
        {
            auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformAstar(
                rollout_graph, s->current_location, goal, &distanceHeuristic, true);

            return result.first[1];
   
        }

        void updateGraph(GraphD& rollout_graph, Observation obs)
        {
            using namespace arc_dijkstras;
            GraphEdge& e = rollout_graph.getEdge(obs.from, obs.to);
            e.setValidity(obs.succeeded() ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID); 
        }
        
        double rolloutOptimistic(GraphD& rollout_graph, State* s)
        {
            double rollout_cost = 0;
            while(s->current_location != goal)
            {
                Action a = optimisticPath(rollout_graph, s);
                Observation ob = transition(s, a, rollout_cost);
                updateGraph(rollout_graph, ob);
            }
            return rollout_cost;
        }

        Observation transition(State* s, Action a, double& accumulated_cost)
        {
            Location cur = s->current_location;
            double b = s->getBlockage(cur, a);
            double weight = s->graph->getEdge(cur, a).getWeight();
            Observation ob(cur, a, b);

            if(ob.succeeded())
            {
                s->current_location = a;
                accumulated_cost += weight;
            }
            else
            {
                accumulated_cost += 2 * b * weight;
            }
            return ob;
        }

    };
}

#endif
