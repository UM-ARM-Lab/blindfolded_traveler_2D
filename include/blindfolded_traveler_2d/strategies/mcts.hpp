#ifndef BTP_MCTS_HPP
#define BTP_MCTS_HPP

#include "strategies/strategy.hpp"
#include "beliefs/obstacle_distribution.hpp"

namespace BTP
{
    namespace MCTS
    {
        class StateNode;

        class ActionNode
        {
        public:
            double summed_cost;
            int visit_count;
            std::vector<StateNode*> children;
            StateNode* parent;
            Action action;

            double getCostEstimate()
            {
                return summed_cost/visit_count;
            }
        };

        class StateNode
        {
        public:
            ObstacleBelief bel;
            std::vector<ActionNode*> children;
            int visit_count;
            ActionNode* parent;

            StateNode(ObstacleBelief bel) : bel(bel) {};
        };

        
        
        class Tree
        {
        public:
            StateNode* root;

            Tree(ObstacleBelief bel)
            {
                root = new StateNode(bel);
            }

            ~Tree()
            {
                deleteSubtree(root);
            }

            void deleteSubtree(StateNode* root)
            {
                for(ActionNode* child:root->children)
                {
                    deleteSubtree(child);
                }
                delete root;
            }

            void deleteSubtree(ActionNode* root)
            {
                for(StateNode* child:root->children)
                {
                    deleteSubtree(child);
                }
                delete root;
            }

            ActionNode* addNode(StateNode* parent, Action a)
            {
                ActionNode* child = new ActionNode();
                parent->children.push_back(child);
                child->parent = parent;
                child->action = a;
                return child;
            }

            StateNode* addNode(ActionNode* parent, ObstacleBelief bel)
            {
                StateNode* child = new StateNode(bel);
                parent->children.push_back(child);
                child->parent = parent;
                return child;
            }
        };


        class MCTS
        {
        public:
            Tree tree;
            std::mt19937 rng;
            Location goal;
            
            MCTS(ObstacleBelief bel, Location goal) : tree(bel), goal(goal)
            {
                addActions(tree.root);
                tree.root->visit_count = 1;
            }

            std::vector<Action> getActions(StateNode* n)
            {
                std::vector<Action> actions;
                for(const auto& e : n->bel.getNode(n->bel.cur).getEdges)
                {
                    actions.push_back(e.getToIndex);
                }
                return actions;
            }

            void addActions(StateNode* n)
            {
                for(Action a : getActions(n))
                {
                    tree.addNode(n, a);
                }
            }

            StateNode* expand(ActionNode* parent, const ObstacleBelief& s)
            {
                StateNode* child = tree.addNode(parent, s);
                addActions(child);
                return child;
            }

            arc_helpers::AstarResult heuristicPath(const ObstacleBelief &bel)
            {
                auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformAstar(
                    bel.graph, bel.cur, goal, &distanceHeuristic, true);

                return result.first[1];
            }

            Action fastPolicy(const ObstacleBelief& bel)
            {
                auto result = heuristicPath(bel);
                return result.first[1];
            }

            Observation transition(State* s, Action a)
            {
                Location cur = s->getLocation()
                double b = s->getBlockage(cur, a);
                double weight = s->graph.getEdge(cur, a).getWeight();
                Observation ob(cur, a, b);
                double cost;

                if(ob.succeeded())
                {
                    s->current_location = a;
                    cost = weight;
                }
                else
                {
                    cost = 2 * b * weight;
                }
                
            }

            void rollout()
            {
                ObstacleBelief& bel = tree.root->bel;
                std::shared_ptr<State> s = bel.sample(rng);
                StateNode* node = tree.root;
                std::vector<double> costs;
                bool in_tree = true;

                while(s->cur != goal)
                {
                    ActionNode* an = selectPromisingAction(node);
                    transition(s.get(), an->action);
                }
            }
        };
    }

    
    class MCTS_Strategy : public Strategy
    {
    public:
        ObstacleBelief bel;

    public:
        MCTS(GraphD graph, Location goal, ObstacleBelief bel) :
            Strategy(graph, goal), bel(bel)
        {
            name = "MCTS";
        }
    };

    virtual Action getNextAction(Location current, Observation obs) override;

}

#endif
