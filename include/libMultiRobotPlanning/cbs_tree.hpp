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
        int id;
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
    class btree{
        public:
            btree(){
                root=NULL;
            }
            ~btree(){
                destroy_tree();
            }

            treeNode<HighLevelNode, Conflict>* insert(int *id, HighLevelNode* highLevelNodeTree, treeNode<HighLevelNode, Conflict> *node ){
                return insertPriv(id, highLevelNodeTree, node);
            }
            treeNode<HighLevelNode, Conflict>* insertRoot(int *id, HighLevelNode* highLevelNodeTree){
                return insertRootPriv(id, highLevelNodeTree);
            }

            treeNode<HighLevelNode, Conflict>* search(int *key){
                return searchPriv(key);
            }
            void destroy_tree(){
                destroy_tree(root);
                tree_leafs.clear();
            }

        private:
            void destroy_tree(treeNode<HighLevelNode, Conflict>* leaf);

            treeNode<HighLevelNode, Conflict>* insertRootPriv(int *id, HighLevelNode* highLevelNodeTree);

            treeNode<HighLevelNode, Conflict>* insertPriv(int *id, HighLevelNode* highLevelNodeTree, treeNode<HighLevelNode, Conflict>* leaf);

            treeNode<HighLevelNode, Conflict>* searchPriv(int *key);
            
            std::vector<treeNode<HighLevelNode,Conflict>*> tree_leafs;
            treeNode<HighLevelNode, Conflict>* root;

    };

    template<typename HighLevelNode, typename Conflict>
    void btree<HighLevelNode, Conflict>::destroy_tree(treeNode<HighLevelNode, Conflict>* leaf){
                if(leaf!=NULL)
        {
            destroy_tree(leaf->child_left);
            destroy_tree(leaf->child_right);
            delete leaf->highLevelNodeTree;
            delete leaf;
        }
    }

    template<typename HighLevelNode, typename Conflict>
    treeNode<HighLevelNode, Conflict>* btree<HighLevelNode, Conflict>::insertRootPriv(int *id, HighLevelNode* highLevelNodeTree){
        if(root == NULL){
            root = new treeNode<HighLevelNode, Conflict>();
            root->id = *id;
            root->highLevelNodeTree = highLevelNodeTree;
            root->conflict = Conflict();
            root->parent = NULL;
            root->child_left = NULL;
            root->child_right = NULL;
        }
        //tree_leafs.push_back(root);
        return root;
    }

    template<typename HighLevelNode, typename Conflict>
    treeNode<HighLevelNode, Conflict>* btree<HighLevelNode, Conflict>::insertPriv(int *id, HighLevelNode* highLevelNodeTree, treeNode<HighLevelNode, Conflict>* leaf){
        if(leaf->child_left == NULL){
            leaf->child_left = new treeNode<HighLevelNode, Conflict>();
            leaf->child_left->id =*id;
            leaf->child_left->highLevelNodeTree = highLevelNodeTree;
            leaf->child_left->conflict = Conflict();
            leaf->child_left->parent = leaf;
            leaf->child_left->child_left = NULL;
            leaf->child_left->child_right = NULL;
            //remove leaf from list
            // typename std::vector<treeNode<HighLevelNodeTree>*>::iterator it = std::find(tree_leafs.begin(), tree_leafs.end(),leaf);
            // if (it != tree_leafs.end()){
            //     tree_leafs.erase(it);
            // }
            //add leaf to list
            //tree_leafs.push_back(leaf->child_left);
            
            return leaf->child_left;
        }else{
            leaf->child_right = new treeNode<HighLevelNode, Conflict>();
            leaf->child_right->id =*id;
            leaf->child_right->highLevelNodeTree = highLevelNodeTree;
            leaf->child_right->conflict =  Conflict();
            leaf->child_right->parent = leaf;
            leaf->child_right->child_left = NULL;
            leaf->child_right->child_right = NULL;
            //add leaf to list
            //tree_leafs.push_back(leaf->child_right);

            return leaf->child_right;
        }
    }

    template<typename HighLevelNode, typename Conflict>
    treeNode<HighLevelNode, Conflict>* btree<HighLevelNode, Conflict>::searchPriv(int *key){
        //search only in leafs! not all tree (because only them can be in open)
        for(typename std::vector<treeNode<HighLevelNode, Conflict>*>::iterator it = tree_leafs.begin(); it != tree_leafs.end(); ++it) {
            if((*(it))->id == *key){
                return *it;
            }
        }
        return NULL;
    }
}  // namespace libMultiRobotPlanning