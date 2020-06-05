#pragma once

#include <map>

#include "a_star.hpp"
namespace libMultiRobotPlanning {
    template <typename State, typename Action, typename Cost, typename Conflict, typename Constraints>
    struct treeNode{
        int id;
        std::vector<PlanResult<State, Action, Cost> > solution;
        std::vector<Constraints> constraints;
        Conflict conflict;
        treeNode* parent;
        treeNode* child_left;
        treeNode* child_right;
    };



    template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
    class btree{
        public:
            btree(){
                root=NULL;
            }
            ~btree(){
                destroy_tree();
            }

            void insert(int *id, std::vector<PlanResult<State, Action, Cost> > *solution, std::vector<Constraints> *constraints, treeNode<State, Action, Cost, Conflict, Constraints> *node ){
                insertPriv(id, solution, constraints, node);
            }
            treeNode<State, Action, Cost, Conflict, Constraints>* insertRoot(int *id, std::vector<PlanResult<State, Action, Cost> > *solution, std::vector<Constraints> *constraints){
                return insertRootPriv(id, solution, constraints);
            }

            treeNode<State, Action, Cost, Conflict, Constraints>* search(int *key){
                return searchPriv(key);
            }
            void destroy_tree(){
                destroy_tree(root);
                tree_leafs.clear();
            }

        private:
            void destroy_tree(treeNode<State, Action, Cost, Conflict, Constraints>* leaf){
                if(leaf!=NULL)
                {
                    destroy_tree(leaf->child_left);
                    destroy_tree(leaf->child_right);
                    delete leaf;
                }
            }

            treeNode<State, Action, Cost, Conflict, Constraints>* insertRootPriv(int *id, std::vector<PlanResult<State, Action, Cost> > *solution, std::vector<Constraints> *constraints){
                if(root == NULL){
                    root = new treeNode<State, Action, Cost, Conflict, Constraints>();
                    root->id = *id;
                    root->solution = *solution;
                    root->constraints = *constraints;
                    root->conflict = Conflict();
                    root->parent = NULL;
                    root->child_left = NULL;
                    root->child_right = NULL;
                }
                tree_leafs.push_back(root);
                return root;
            }
            void insertPriv(int *id, std::vector<PlanResult<State, Action, Cost> > *solution, std::vector<Constraints> *constraints, treeNode<State, Action, Cost, Conflict, Constraints>* leaf){
                if(leaf->child_left == NULL){
                    leaf->child_left = new treeNode<State, Action, Cost, Conflict, Constraints>();
                    leaf->child_left->id =*id;
                    leaf->child_left->solution = *solution;
                    leaf->child_left->constraints = *constraints;
                    leaf->child_left->conflict = Conflict();
                    leaf->child_left->parent = leaf;
                    leaf->child_left->child_left = NULL;
                    leaf->child_left->child_right = NULL;
                    //remove leaf from list
                    typename std::vector<treeNode<State, Action, Cost, Conflict, Constraints>*>::iterator it = std::find(tree_leafs.begin(), tree_leafs.end(),leaf);
                    if (it != tree_leafs.end()){
                        tree_leafs.erase(it);
                    }
                    //add leaf to list
                    tree_leafs.push_back(leaf->child_left);
                }else{
                    leaf->child_right = new treeNode<State, Action, Cost, Conflict, Constraints>();
                    leaf->child_right->id =*id;
                    leaf->child_right->solution = *solution;
                    leaf->child_right->constraints = *constraints;
                    leaf->child_right->conflict =  Conflict();
                    leaf->child_right->parent = leaf;
                    leaf->child_right->child_left = NULL;
                    leaf->child_right->child_right = NULL;
                    //add leaf to list
                    tree_leafs.push_back(leaf->child_right);
                }
                return;
            }

            treeNode<State, Action, Cost, Conflict, Constraints>* searchPriv(int *key){
                //search only in leafs! not all tree (because only them can be in open)
                for(typename std::vector<treeNode<State, Action, Cost, Conflict, Constraints>*>::iterator it = tree_leafs.begin(); it != tree_leafs.end(); ++it) {
                    if((*(it))->id == *key){
                        return *it;
                    }
                }
                return NULL;
            }
            
            std::vector<treeNode<State, Action, Cost, Conflict, Constraints>*> tree_leafs;
            treeNode<State, Action, Cost, Conflict, Constraints>* root;

    };
}  // namespace libMultiRobotPlanning