#pragma once

#include <map>

#include "a_star.hpp"
namespace libMultiRobotPlanning {
    template <class T> 
    struct my_less {
        bool operator() (const T& x, const T& y) const {return *x<*y;}
    };
    template <typename HighLevelNode, typename Conflict>
    struct treeNode{
        //int id;
        HighLevelNode* highLevelNodeTree;
        Conflict conflict;
        typename boost::heap::d_ary_heap< treeNode<HighLevelNode, Conflict>*, boost::heap::arity<2>, 
                                     boost::heap::mutable_<true>,boost::heap::compare<my_less<treeNode<HighLevelNode, Conflict>*>> >::handle_type
        handle;
        treeNode* parent;
        treeNode* child_left;
        treeNode* child_right;
        bool operator<(const treeNode& n) const {
            // if (cost != n.cost)
            return *(highLevelNodeTree) < *(n.highLevelNodeTree);
            // return id > n.id;
        }

    };
    template <typename HighLevelNode, typename Conflict>
    struct compare_set{
        bool operator()(const treeNode<HighLevelNode,Conflict> *lhs, const treeNode<HighLevelNode,Conflict>  *rhs) const {
            if (lhs->highLevelNodeTree->constraints < rhs->highLevelNodeTree->constraints){
                return true;
            }
            return false;
        }
    };
    template <typename HighLevelNode, typename Conflict, typename State>
    class btree{
        public:
            btree(){
                root=NULL;
            }
            ~btree(){
                destroy_tree();
            }

            treeNode<HighLevelNode, Conflict>* insert(HighLevelNode* highLevelNodeTree, treeNode<HighLevelNode, Conflict> *node ){
                return insertPriv(highLevelNodeTree, node);
            }
            treeNode<HighLevelNode, Conflict>* insertRoot(HighLevelNode* highLevelNodeTree){
                return insertRootPriv(highLevelNodeTree);
            }

            void destroy_tree(){
                destroy_tree(root);
                new_tree_leafs.clear();
            }


            std::vector<treeNode<HighLevelNode,Conflict>*> PreorderPrunTreeTraveling(size_t agentId, int timeStep, int& id, std::size_t agentNumber,std::vector<State> &startStates){
                bool simple_node_with_zero_constraints_exist = false;
                PreorderPrunTreeTravelingPriv(root, agentId, timeStep,id, agentNumber, startStates, simple_node_with_zero_constraints_exist);
                std::set<treeNode<HighLevelNode,Conflict>*, compare_set<HighLevelNode,Conflict>> node_set;
                for (auto const& n : new_tree_leafs){
                    node_set.insert(n);
                }
                new_tree_leafs.clear();
                for(auto const& n : node_set){
                    new_tree_leafs.push_back(n);
                }
                return new_tree_leafs;
            }
            treeNode<HighLevelNode, Conflict>* GetRoot(){
                return root;
            }
            void nullToRoot();

            bool compare(const treeNode<HighLevelNode,Conflict> *lhs, const treeNode<HighLevelNode,Conflict>  *rhs) const {
                if (lhs->highLevelNodeTree->constraints == rhs->highLevelNodeTree->constraints){
                    return true;
                }
                return false;
            }

        private:
            void destroy_tree(treeNode<HighLevelNode, Conflict>* leaf);

            treeNode<HighLevelNode, Conflict>* insertRootPriv(HighLevelNode* highLevelNodeTree);

            treeNode<HighLevelNode, Conflict>* insertPriv(HighLevelNode* highLevelNodeTree, treeNode<HighLevelNode, Conflict>* leaf);

            //Prun node right and left childs 
            void prunSubTreePriv(treeNode<HighLevelNode, Conflict> *node);

            void PreorderPrunTreeTravelingPriv(treeNode<HighLevelNode, Conflict>* node, size_t agentId, int timeStep, int& id,std::size_t agentNumber,std::vector<State> &startStates, bool &simple_node_with_zero_constraints_exist);

            std::vector<treeNode<HighLevelNode,Conflict>*> new_tree_leafs;

            treeNode<HighLevelNode, Conflict>* root;
            

    };

    template<typename HighLevelNode, typename Conflict, typename State>
    void btree<HighLevelNode, Conflict, State>::destroy_tree(treeNode<HighLevelNode, Conflict>* leaf){
                if(leaf!=NULL)
        {
            destroy_tree(leaf->child_left);
            destroy_tree(leaf->child_right);
            delete leaf->highLevelNodeTree;
            delete leaf;
        }
    }

    template<typename HighLevelNode, typename Conflict, typename State>
    treeNode<HighLevelNode, Conflict>* btree<HighLevelNode, Conflict, State>::insertRootPriv(HighLevelNode* highLevelNodeTree){
        if(root == NULL){
            root = new treeNode<HighLevelNode, Conflict>();
            root->highLevelNodeTree = highLevelNodeTree;
            root->conflict = Conflict();
            root->parent = NULL;
            root->child_left = NULL;
            root->child_right = NULL;
        }
        
        return root;
    }

    template<typename HighLevelNode, typename Conflict, typename State>
    treeNode<HighLevelNode, Conflict>* btree<HighLevelNode, Conflict, State>::insertPriv(HighLevelNode* highLevelNodeTree, treeNode<HighLevelNode, Conflict>* leaf){
        if(leaf->child_left == NULL){
            leaf->child_left = new treeNode<HighLevelNode, Conflict>();
            leaf->child_left->highLevelNodeTree = highLevelNodeTree;
            leaf->child_left->conflict = Conflict();
            leaf->child_left->parent = leaf;
            leaf->child_left->child_left = NULL;
            leaf->child_left->child_right = NULL;
            
            return leaf->child_left;
        }else{
            leaf->child_right = new treeNode<HighLevelNode, Conflict>();
            leaf->child_right->highLevelNodeTree = highLevelNodeTree;
            leaf->child_right->conflict =  Conflict();
            leaf->child_right->parent = leaf;
            leaf->child_right->child_left = NULL;
            leaf->child_right->child_right = NULL;
            return leaf->child_right;
        }
    }


    template<typename HighLevelNode, typename Conflict, typename State>
    void btree<HighLevelNode, Conflict, State>::prunSubTreePriv(treeNode<HighLevelNode, Conflict> *node){
        if(NULL != node->child_left){
            destroy_tree(node->child_left);
            node->child_left = NULL;
        }
        if(NULL != node->child_right){
            destroy_tree(node->child_right);
            node->child_right = NULL;
        }
    }

    //return a vector of the new CT leaves (to be inserted to a new open list)
    template<typename HighLevelNode, typename Conflict, typename State>
    void btree<HighLevelNode, Conflict, State>::PreorderPrunTreeTravelingPriv(treeNode<HighLevelNode, Conflict>* node, size_t agentId, int timeStep, int& id,std::size_t agentNumber,std::vector<State> &startStates, bool &simple_node_with_zero_constraints_exist){
        if (node == NULL) 
            return; 
        if(node->conflict.time > timeStep){
            if ((node->conflict.agent1 == agentId) || (node->conflict.agent2 == agentId) ){
                prunSubTreePriv(node);
                if( true == UpdateNodes(node, id, timeStep, agentNumber, startStates, simple_node_with_zero_constraints_exist)){
                        new_tree_leafs.push_back(node);     
                }
                return;
            }
        }
        if(node->child_left == NULL && node->child_right == NULL){
            if( true == UpdateNodes(node, id, timeStep, agentNumber, startStates, simple_node_with_zero_constraints_exist)){
                    new_tree_leafs.push_back(node);     
            }
        }
        
        /* then recur on the left subtree */
        PreorderPrunTreeTravelingPriv(node->child_left, agentId, timeStep,id, agentNumber, startStates,simple_node_with_zero_constraints_exist);  
    
        /* now recur on the right subtree */
        PreorderPrunTreeTravelingPriv(node->child_right, agentId, timeStep, id, agentNumber, startStates,simple_node_with_zero_constraints_exist); 
    }

    template<typename HighLevelNode, typename Conflict, typename State>
    void btree<HighLevelNode, Conflict, State>::nullToRoot(){
        root = NULL;
    }



}  // namespace libMultiRobotPlanning