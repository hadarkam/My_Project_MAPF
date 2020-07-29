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


            std::vector<treeNode<HighLevelNode,Conflict>*> PreorderPrunTreeTraveling(size_t agentId, int timeStep){
                PreorderPrunTreeTravelingPriv(root, agentId, timeStep);
                return new_tree_leafs;
            }
            treeNode<HighLevelNode, Conflict>* GetRoot(){
                return root;
            }
            void nullToRoot();

        private:
            void destroy_tree(treeNode<HighLevelNode, Conflict>* leaf);

            treeNode<HighLevelNode, Conflict>* insertRootPriv(HighLevelNode* highLevelNodeTree);

            treeNode<HighLevelNode, Conflict>* insertPriv(HighLevelNode* highLevelNodeTree, treeNode<HighLevelNode, Conflict>* leaf);

            //Prun node right and left childs 
            void prunSubTreePriv(treeNode<HighLevelNode, Conflict> *node);

            void PreorderPrunTreeTravelingPriv(treeNode<HighLevelNode, Conflict>* node, size_t agentId, int timeStep);

            std::vector<treeNode<HighLevelNode,Conflict>*> new_tree_leafs;

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
    treeNode<HighLevelNode, Conflict>* btree<HighLevelNode, Conflict>::insertRootPriv(HighLevelNode* highLevelNodeTree){
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

    template<typename HighLevelNode, typename Conflict>
    treeNode<HighLevelNode, Conflict>* btree<HighLevelNode, Conflict>::insertPriv(HighLevelNode* highLevelNodeTree, treeNode<HighLevelNode, Conflict>* leaf){
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


    template<typename HighLevelNode, typename Conflict>
    void btree<HighLevelNode, Conflict>::prunSubTreePriv(treeNode<HighLevelNode, Conflict> *node){
        if(NULL != node->child_left){
            destroy_tree(node->child_left);
            node->child_left = NULL;
        }
        if(NULL != node->child_right){
            destroy_tree(node->child_right);
            node->child_right = NULL;
        }
    }

    //return vector of the new CT leaves (to be inserted to a new open list)
    template<typename HighLevelNode, typename Conflict>
    void btree<HighLevelNode, Conflict>::PreorderPrunTreeTravelingPriv(treeNode<HighLevelNode, Conflict>* node, size_t agentId, int timeStep){
        if (node == NULL) 
            return; 
        if(node->conflict.time > timeStep){
            if ((node->conflict.agent1 == agentId) || (node->conflict.agent2 == agentId) ){
                prunSubTreePriv(node);
                new_tree_leafs.push_back(node);
                return;
            }
        }
        if(node->child_left == NULL && node->child_right == NULL){
            new_tree_leafs.push_back(node);
        }
        
        /* then recur on the left subtree */
        PreorderPrunTreeTravelingPriv(node->child_left, agentId, timeStep);  
    
        /* now recur on the right subtree */
        PreorderPrunTreeTravelingPriv(node->child_right, agentId, timeStep); 
    }

    template<typename HighLevelNode, typename Conflict>
    void btree<HighLevelNode, Conflict>::nullToRoot(){
        root = NULL;
    }

}  // namespace libMultiRobotPlanning