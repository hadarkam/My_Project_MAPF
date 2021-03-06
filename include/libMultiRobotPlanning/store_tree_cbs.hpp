#pragma once

#include <map>

#include "a_star.hpp"
#include <libMultiRobotPlanning/cbs_tree.hpp>

namespace libMultiRobotPlanning {

/*!
  \example cbs.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right
  actions
*/

/*! \brief Conflict-Based-Search (CBS) algorithm to solve the Multi-Agent
Path-Finding (MAPF) problem

This class implements the Conflict-Based-Search (CBS) algorithm.
This algorithm can find collision-free path for multiple agents with start and
goal locations
given for each agent.
CBS is a two-level search. On the low-level, A* is used to find paths for
individual agents (ideally using a perfect heuristic).
The high-level is a tree-search that resolves conflicts between agents as they
occur, earliest conflict-time first.
CBS is optimal with respect to the sum-of-individual costs.

Details of the algorithm can be found in the following paper:\n
Guni Sharon, Roni Stern, Ariel Felner, Nathan R. Sturtevant:\n
"Conflict-based search for optimal multi-agent pathfinding". Artif. Intell. 219:
40-66 (2015)\n
https://doi.org/10.1016/j.artint.2014.11.006

The underlying A* can either use a fibonacci heap, or a d-ary heap.
The latter is the default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap
instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Conflict Custom conflict description. A conflict needs to be able to be
transformed into a constraint.
\tparam Constraints Custom constraint description. The Environment needs to be
able to search on the low-level while taking the constraints into account.
\tparam Environment This class needs to provide the custom logic. In particular,
it needs to support the following functions:
  - `void setLowLevelContext(size_t agentIdx, const Constraints* constraints)`\n
    Set the current context to a particular agent with the given set of
constraints

  - `Cost admissibleHeuristic(const State& s)`\n
    Admissible heuristic. Needs to take current context into account.

  - `bool isSolution(const State& s)`\n
    Return true if the given state is a goal state for the current agent.

  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int>
>& neighbors)`\n
    Fill the list of neighboring state for the given state s and the current
agent.

  - `bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >&
solution, Conflict& result)`\n
    Finds the first conflict for the given solution for each agent. Return true
if a conflict was found and false otherwise.

  - `void createConstraintsFromConflict(const Conflict& conflict,
std::map<size_t, Constraints>& constraints)`\n
    Create a list of constraints for the given conflict.

  - `void onExpandHighLevelNode(Cost cost)`\n
    This function is called on every high-level expansion and can be used for
statistical purposes.

  - `void onExpandLowLevelNode(const State& s, Cost fScore, Cost gScore)`\n
    This function is called on every low-level expansion and can be used for
statistical purposes.
*/
template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class TREE_CBS {
 public:

  struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;

    bool operator<(const HighLevelNode& n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };

  TREE_CBS(Environment& environment) : m_env(environment) {}

  bool search(const std::vector<State>& initialStates, std::vector<PlanResult<State, Action, Cost> >& solution, btree< HighLevelNode, Conflict, State> *cbs_tree,
  std::vector<treeNode<HighLevelNode,Conflict>*>& treeNodeVector, int id, int agentNumber) {
    HighLevelNode *start = new HighLevelNode();

    if (treeNodeVector.empty()){
      //regular (no initial states)
      start->solution.resize(initialStates.size());
      start->constraints.resize(initialStates.size());
      start->cost = 0;
      start->id = 0;

      for (size_t i = 0; i < initialStates.size(); ++i) {

        LowLevelEnvironment llenv(m_env, i, start->constraints[i]);
        LowLevelSearch_t lowLevel(llenv);
        bool success = lowLevel.search(initialStates[i], start->solution[i]);
        if (!success) {
          return false;
        }
        // }
        start->cost += start->solution[i].cost;
      }
    }

    // std::priority_queue<HighLevelNode> open;
    typename boost::heap::d_ary_heap<treeNode< HighLevelNode, Conflict>*, boost::heap::arity<2>,
                                    boost::heap::mutable_<true>,boost::heap::compare<my_less<treeNode<HighLevelNode, Conflict>*>> >
        open; //note: implemented as a max heap data structure

    treeNode<HighLevelNode, Conflict>* tree_root;
    if (cbs_tree->GetRoot() == NULL || treeNodeVector.empty()){
      //initialize binary tree and insert root (=start)
      //btree<State,Action,Cost, Conflict, Constraints, Environment> *cbs_tree = new btree<State,Action,Cost, Conflict, Constraints, Environment>();
      //root's conflict is null until we find the first conflict (we will split with)
      tree_root = cbs_tree->insertRoot(start);
    }
    

    if (treeNodeVector.empty()){
      //This is the first phase (open has only one new root node)
      auto handle = open.push(tree_root); 
      (*handle)->handle = handle;
    }else{
      Timer insert_time;
      //This is the second phase (insert to open all nodes from treeNodeVector)
      //int counter = 1;
      PlanResult<State, Action, Cost> agentSolution;
      //beg
      Constraints constraint;
      LowLevelEnvironment llenv(m_env, agentNumber, constraint);
      LowLevelSearch_t lowLevel(llenv);
      bool success = lowLevel.search(initialStates[agentNumber], agentSolution);
      if (!success) {
        return false;
      }
      //end
      for (auto const& treeNode : treeNodeVector){
        
        treeNode->highLevelNodeTree->solution[agentNumber] = agentSolution;
        treeNode->highLevelNodeTree->cost +=agentSolution.cost;
        // update the id to make sure all nodes in the new tree have different id
        //note: only the leaves! the inner leaves id remain the same for now (should change it?)
        treeNode->highLevelNodeTree->id = id;
        id++;
        // push node to open list
        auto handle = open.push(treeNode); 
        (*handle)->handle = handle;
      }
      insert_time.stop();
      std::cout << "time to insert all reused nodes to open list: " << insert_time.elapsedSeconds() << std::endl;
    }

    solution.clear();
    //int id = 1;
    Timer time_open_list;
    //to remove:
    int number_of_developed_nodes = 0;
    while (!open.empty()) {
      treeNode<HighLevelNode, Conflict>* P = open.top();
      m_env.onExpandHighLevelNode(P->highLevelNodeTree->cost);
      // std::cout << "expand: " << P << std::endl;

      open.pop();
     
      Conflict conflict;
      if (!m_env.getFirstConflict(P->highLevelNodeTree->solution, conflict)) {
        std::cout << "done; cost: " << P->highLevelNodeTree->cost << std::endl;
        solution = P->highLevelNodeTree->solution;

        time_open_list.stop();
        std::cout << "time with open list: " << time_open_list.elapsedSeconds() << std::endl;
        return true;
      }
      //insert conflict to tree_node (P->conflict represent the conflict that makes this node to split!)
      P->conflict = conflict;

      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
        
        size_t i = c.first;

        HighLevelNode *newNode = new HighLevelNode();
        newNode->constraints = P->highLevelNodeTree->constraints;
        newNode->cost = P->highLevelNodeTree->cost;
        newNode->solution = P->highLevelNodeTree->solution;
        newNode->id = id;

        //(optional) check that this constraint was not included already
        // std::cout << newNode.constraints[i] << std::endl;
        // std::cout << c.second << std::endl;
        assert(!newNode->constraints[i].overlap(c.second));

        newNode->constraints[i].add(c.second);

        newNode->cost -= newNode->solution[i].cost;

        LowLevelEnvironment llenv(m_env, i, newNode->constraints[i]);
        LowLevelSearch_t lowLevel(llenv);
        bool success = lowLevel.search(initialStates[i], newNode->solution[i]);

        newNode->cost += newNode->solution[i].cost;

        //to remove: (for debuge)
        number_of_developed_nodes++;


        //insert newNode to tree
        treeNode<HighLevelNode, Conflict> *new_tree_node = cbs_tree->insert(newNode, P);

        if (success) {
          // std::cout << "  success. cost: " << newNode.cost << std::endl;
          auto handle = open.push(new_tree_node);
          (*handle)->handle = handle;
        }
        ++id;
      }
    }

    return false;
  }

  


   

  private:
  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(const State& s) { return m_env.isSolution(s); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost> >& neighbors) {
      m_env.getNeighbors(s, neighbors);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      // std::cout << "LL expand: " << s << std::endl;
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
      // std::cout << "LL discover: " << s << std::endl;
      // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }

   private:
    Environment& m_env;
    // size_t m_agentIdx;
    // const Constraints& m_constraints;
  };


 private:
  Environment& m_env;
  typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
