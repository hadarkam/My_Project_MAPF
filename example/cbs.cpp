#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/cbs_tree.hpp>

#include <libMultiRobotPlanning/cbs.hpp>
#include "timer.hpp"
#include <libMultiRobotPlanning/store_tree_cbs.hpp>

using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::TREE_CBS;
using libMultiRobotPlanning::treeNode;
using libMultiRobotPlanning::btree;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

struct State {
  State(int time, int x, int y) : time(time), x(x), y(y) {}

  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  int time;
  int x;
  int y;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

///
enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:
      os << "Up";
      break;
    case Action::Down:
      os << "Down";
      break;
    case Action::Left:
      os << "Left";
      break;
    case Action::Right:
      os << "Right";
      break;
    case Action::Wait:
      os << "Wait";
      break;
  }
  return os;
}

///

struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int x1;
  int y1;
  int x2;
  int y2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
  int time;
  int x;
  int y;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  int time;
  int x1;
  int y1;
  int x2;
  int y2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
  }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::set<VertexConstraint> vertexConstraints;
  std::set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) {
    std::vector<VertexConstraint> vertexIntersection;
    std::vector<EdgeConstraint> edgeIntersection;
    std::set_intersection(vertexConstraints.begin(), vertexConstraints.end(),
                          other.vertexConstraints.begin(),
                          other.vertexConstraints.end(),
                          std::back_inserter(vertexIntersection));
    std::set_intersection(edgeConstraints.begin(), edgeConstraints.end(),
                          other.edgeConstraints.begin(),
                          other.edgeConstraints.end(),
                          std::back_inserter(edgeIntersection));
    return !vertexIntersection.empty() || !edgeIntersection.empty();
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
  //my:
  friend bool operator==(const Constraints& l, const Constraints& r) {
    if (l.edgeConstraints == r.edgeConstraints){
      if(l.vertexConstraints == r.vertexConstraints){
        return true;
      }
    }
    return false;
  }
  //my:
  friend bool operator<(const Constraints& l, const Constraints& r) {
    if (l.vertexConstraints < r.vertexConstraints){
      return true;
    }else if(l.vertexConstraints == r.vertexConstraints){
      if (l.edgeConstraints < r.edgeConstraints){
        return true;
      }
    }
    return false;
  }

};

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

///
class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {
    // computeHeuristic();
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const State& s) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
    return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }

  bool isSolution(const State& s) {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1));
      }
    }
  }

  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int> >& solution,
      Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const State& s1, const State& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }
#if 0
  // We use another A* search for simplicity
  // we compute the shortest path to each goal by using the fact that our getNeighbor function is
  // symmetric and by not terminating the AStar search until the queue is empty
  void computeHeuristic()
  {
    class HeuristicEnvironment
    {
    public:
      HeuristicEnvironment(
        size_t dimx,
        size_t dimy,
        const std::unordered_set<Location>& obstacles,
        std::vector<int>* heuristic)
        : m_dimx(dimx)
        , m_dimy(dimy)
        , m_obstacles(obstacles)
        , m_heuristic(heuristic)
      {
      }

      int admissibleHeuristic(
        const Location& s)
      {
        return 0;
      }

      bool isSolution(
        const Location& s)
      {
        return false;
      }

      void getNeighbors(
        const Location& s,
        std::vector<Neighbor<Location, Action, int> >& neighbors)
      {
        neighbors.clear();

        {
          Location n(s.x-1, s.y);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Left, 1));
          }
        }
        {
          Location n(s.x+1, s.y);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Right, 1));
          }
        }
        {
          Location n(s.x, s.y+1);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Up, 1));
          }
        }
        {
          Location n(s.x, s.y-1);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Down, 1));
          }
        }
      }

      void onExpandNode(
        const Location& s,
        int fScore,
        int gScore)
      {
      }

      void onDiscover(
        const Location& s,
        int fScore,
        int gScore)
      {
        (*m_heuristic)[s.x + m_dimx * s.y] = gScore;
      }

    private:
      bool stateValid(
        const Location& s)
      {
        return    s.x >= 0
               && s.x < m_dimx
               && s.y >= 0
               && s.y < m_dimy
               && m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end();
      }

    private:
      int m_dimx;
      int m_dimy;
      const std::unordered_set<Location>& m_obstacles;
      std::vector<int>* m_heuristic;

    };

    m_heuristic.resize(m_goals.size());

    std::vector< Neighbor<State, Action, int> > neighbors;

    for (size_t i = 0; i < m_goals.size(); ++i) {
      m_heuristic[i].assign(m_dimx * m_dimy, std::numeric_limits<int>::max());
      HeuristicEnvironment henv(m_dimx, m_dimy, m_obstacles, &m_heuristic[i]);
      AStar<Location, Action, int, HeuristicEnvironment> astar(henv);
      PlanResult<Location, Action, int> dummy;
      astar.search(m_goals[i], dummy);
      m_heuristic[i][m_goals[i].x + m_dimx * m_goals[i].y] = 0;
    }
  }
#endif
 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  std::vector<Location> m_goals;
  // std::vector< std::vector<int> > m_heuristic;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
};

void WriteSolutionToOutputFile(bool success,std::vector<libMultiRobotPlanning::PlanResult<State, Action, int>> solution
,std::string outputFile, Timer timer,Environment* mapf){
  if (success) {
    std::cout << "Planning successful! " << std::endl;
    int cost = 0;
    int makespan = 0;
    for (const auto& s : solution) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "  highLevelExpanded: " << (*mapf).highLevelExpanded() << std::endl;
    out << "  lowLevelExpanded: " << (*mapf).lowLevelExpanded() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      // std::cout << "Solution for: " << a << std::endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   std::cout << solution[a].states[i].second << ": " <<
      //   solution[a].states[i].first << "->" << solution[a].actions[i].first
      //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
      // }
      // std::cout << solution[a].states.back().second << ": " <<
      // solution[a].states.back().first << std::endl;

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      t: " << state.second << std::endl;
      }
    }
    
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }
}


  int UpdateNodeConstraints(std::vector<Constraints>& constraintsVector, int timeStep){
  //remove from each constraint with time >= timestep a timestep
      int total_number_of_constraints_in_node =0;
      for ( Constraints& constraint : constraintsVector){
          std::set<EdgeConstraint> newEdgeConstraints;
          for(EdgeConstraint edgeConstraint : constraint.edgeConstraints){
              if(edgeConstraint.time > timeStep){ // > : we don't care about "the" timeStep constrain (this is the start point)
                      edgeConstraint.time = edgeConstraint.time - timeStep;
                      newEdgeConstraints.insert(edgeConstraint);
              }
          }
          total_number_of_constraints_in_node += newEdgeConstraints.size();
          constraint.edgeConstraints = newEdgeConstraints;
          std::set<VertexConstraint> newVertexConstraints;
          for(VertexConstraint vertexConstraint : constraint.vertexConstraints){
              if(vertexConstraint.time > timeStep){
                  vertexConstraint.time = vertexConstraint.time - timeStep;
                  newVertexConstraints.insert(vertexConstraint);
              }
          }
          total_number_of_constraints_in_node += newVertexConstraints.size();
          constraint.vertexConstraints = newVertexConstraints;
      }
      return total_number_of_constraints_in_node;
  }
  //template<typename HighLevelNode, typename Conflict, typename State>
  bool UpdateNodes(treeNode<TREE_CBS<State, Action, int, Conflict, Constraints, Environment>::HighLevelNode,Conflict>* node, int& id, int timeStep, size_t agentNumber,std::vector<State> &startStates, bool &simple_node_with_zero_constraints_exist){
      //update id
      node->highLevelNodeTree->id = id;
      id++;
      //initialize new cost
      int newCost = 0;
      //check which nodes are relevent
      for( std::size_t i = 0; i < node->highLevelNodeTree->solution.size(); i++){
        libMultiRobotPlanning::PlanResult<State, Action, int>& sol = node->highLevelNodeTree->solution[i];
        if((int)sol.states.size() > timeStep){
          //the agent has not arrived at his goal (before or in the timestep)
          if (sol.states[timeStep].first.x != startStates[i].x || sol.states[timeStep].first.y != startStates[i].y){
            //we eant to remove all nodes from treeNodeVector that is not compatible with the chosen solution from the first phase(the start point at timeStep is not the same)
            return false;
          }
        }
      }
      // update constraints:
      int total_number_of_constraints_in_node = UpdateNodeConstraints((node->highLevelNodeTree->constraints), timeStep);
      if( 0 == total_number_of_constraints_in_node && simple_node_with_zero_constraints_exist==true){
        return false;
      }else if(0 == total_number_of_constraints_in_node){
        simple_node_with_zero_constraints_exist = true;
      }
      // update paths: (delete all places before the timeStep)
      for( std::size_t i = 0; i < node->highLevelNodeTree->solution.size(); i++){
          libMultiRobotPlanning::PlanResult<State, Action, int>& sol = node->highLevelNodeTree->solution[i];
          int solStateSize = sol.states.size();

          if (i == agentNumber){
              //the new path will be calculated in the search phase
              continue;
          }
          
          if(solStateSize <= timeStep){
              //the agent already arrived at his goal (before or in the timestep)
              sol.states.erase(sol.states.begin(), sol.states.begin() + solStateSize-1);
              sol.actions.erase(sol.actions.begin(), sol.actions.begin() + solStateSize-1);
              for(std::pair<State, int>& state : sol.states){
                  state.first.time= 0;
                  state.second = 0;
              }
          }else{
              sol.states.erase(sol.states.begin(), sol.states.begin() + timeStep);
              sol.actions.erase(sol.actions.begin(), sol.actions.begin() + timeStep);
              for(std::pair<State, int>& state : sol.states){
                state.first.time= state.first.time - timeStep;
                state.second = state.second - timeStep;
              }
          }
          int size = sol.states.size() - 1;
          newCost += size;
          sol.cost = size;
          std::pair<State, int>* stateEnd = sol.states.end().base();
          sol.fmin = std::abs(sol.states[0].first.x - stateEnd->first.x ) + std::abs(sol.states[0].first.y - stateEnd->first.y);
      }

      // update cost:
      node->highLevelNodeTree->cost = newCost;
  
      return true;
  }



int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFileBase = "/home/hadar/My_Project_MAPF/debug/outputFileBase.yaml";
  std::string outputFileNew = "/home/hadar/My_Project_MAPF/debug/outputFileNew.yaml";
  std::string secondOutputFileBase = "/home/hadar/My_Project_MAPF/debug/secondOutputFileBase.yaml";
  std::string secondOutputFileNew = "/home/hadar/My_Project_MAPF/debug/secondOutputFileNew.yaml";
  std::string approach;
  std::size_t agentNumber;
  int timeStep;
  int goal_x;
  int goal_y;

  desc.add_options()
      ("help", "produce help message")
      ("input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")
      ("approach", po::value<std::string>(&approach)->required(),
      "naive or pruning approach")
      ("agentNumber", po::value<std::size_t>(&agentNumber)->required(),
      "agent with new goal location")
      ("timeStep", po::value<int>(&timeStep)->required(),
      "the time step in which the agent goal is changing")
      ("x", po::value<int>(&goal_x)->required(),
      "location x of the new goal")
      ("y", po::value<int>(&goal_y)->required(),
      "location y of the new goal");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<Location> obstacles;
  std::vector<Location> goals;
  std::vector<State> startStates;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  //Flags checks:
  //1. The goal is inside the map borders and is not obstacles 
  if(goal_x > dimx || goal_x < 0 || goal_y < 0|| goal_y >dimy){
    std::cout << "The new goal location is out of map borders!" << std::endl;
    return 0; 
  }
  for (const auto& elem: obstacles) {
    if(elem.x == goal_x && elem.y == goal_y){
      std::cout << "The new goal location is an obstacle!" << std::endl;
      return 0;
    }
  }
  //2. The agent number is valid
  if(agentNumber > goals.size()){
    std::cout << "The agentNumber is not valid!" << std::endl;
    return 0;
  }
  //3.timestep >= 0
  if(timeStep < 0){
    std::cout << "The timeStep can not be negative!" << std::endl;
    return 0;
  }
  //4. Check that the new goal is not the same as another agent goal
  for (const auto& elem: goals){
    if(elem.x == goal_x && elem.y == goal_y){
      std::cout << "The goal location is the same as another agent goal!" << std::endl;
      return 0;
    }
  }


  // open csv file to write results
  std::ofstream myfile;
  myfile.open ("results_for_graph.csv",std::ios_base::app);
  std::ofstream myfile2;
  myfile2.open ("results_for_graph_improment_to_number_of_nodes.csv",std::ios_base::app);
  std::ofstream myfile3;
  myfile3.open ("results_for_graph_number_of_constraint_fisrt_level",std::ios_base::app);
  std::ofstream myfile4;
  myfile4.open ("results_for_graph_old_and_new_distance_to_goal",std::ios_base::app);
  std::ofstream myfile5;
  myfile5.open ("results_for_graph__distance_between_goals",std::ios_base::app);
  myfile5 << inputFile << ",";
  myfile5 << sqrt(pow((startStates[agentNumber].x -goal_x),2) + pow((startStates[agentNumber].y -goal_y),2)) << std::endl;
  myfile5.close();
  // return 0 ;


  // todo: compare run time between to different "first phase"

  //************first phase*************// - without saving a tree (regular run)
  // First run of CBS
  Environment mapf(dimx, dimy, obstacles, goals);
  CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);
  std::vector<PlanResult<State, Action, int> > solution;

  Timer timer;
  bool success = cbs.search(startStates, solution, myfile2, agentNumber, timeStep, myfile3); // for enalyse: myfile2, agentNumber, timeStep
  timer.stop();
  std::cout << "base CBS: " << timer.elapsedSeconds() << std::endl;

  WriteSolutionToOutputFile(success,solution,outputFileBase,timer,&mapf);
  //************first phase*************// - without saving a tree (regular run)

  //************first phase - build CT with CBS*************// 
  // First run of CBS
  Environment tree_mapf(dimx, dimy, obstacles, goals);
  TREE_CBS<State, Action, int, Conflict, Constraints, Environment> tree_cbs(tree_mapf);
  std::vector<PlanResult<State, Action, int> > tree_solution;

  btree< TREE_CBS<State, Action, int, Conflict, Constraints, Environment>::HighLevelNode, Conflict, State> *ct_tree = new btree< TREE_CBS<State, Action, int, Conflict, Constraints, Environment>::HighLevelNode, Conflict, State>();

  Timer tree_timer;
  std::vector<treeNode<TREE_CBS<State, Action, int, Conflict, Constraints, Environment>::HighLevelNode,Conflict>*> emptyVector = std::vector<treeNode<TREE_CBS<State, Action, int, Conflict, Constraints, Environment>::HighLevelNode,Conflict>*>();
  bool tree_success = tree_cbs.search(startStates, tree_solution, ct_tree, emptyVector,1,0);
  tree_timer.stop();
  std::cout << "tree CBS: " << tree_timer.elapsedSeconds() << std::endl;

  WriteSolutionToOutputFile(tree_success,tree_solution,outputFileNew,tree_timer,&tree_mapf);
  //************first phase - build CT with CBS*************// 
    std::cout << "difference( tree / base): " << tree_timer.elapsedSeconds() / timer.elapsedSeconds() << std::endl;

  //************second phase- find new paths after "the agent" goal changed*************//

  // for results only:
  int distance_to_old_goal = 0;
  if ((int)(solution[agentNumber].states.size()) > timeStep){
    distance_to_old_goal = std::abs(solution[agentNumber].states[timeStep].first.x - goals[agentNumber].x ) + std::abs(solution[agentNumber].states[timeStep].first.y - goals[agentNumber].y );
  }

  //change the chosen agent goal location
  goals[agentNumber].x= goal_x;
  goals[agentNumber].y= goal_y;
  //change all agents start location using the solution from old CBS
  for (size_t a = 0; a < solution.size(); ++a) {
    if(solution[a].states.size() <= (size_t)timeStep){
      // the agent has arrived at its goal location
      if( a == agentNumber){
        //in this case our agent allready arrived to its old goal but now he needs to go to its new goal
        // Note: if I want him to remain in its old goal then no second phase needs to occur.
        startStates[a].x = solution[a].states.back().first.x;
        startStates[a].y =solution[a].states.back().first.y;
      }else{
        startStates[a].x = goals[a].x;
        startStates[a].y = goals[a].y;
      }
    }else{
    startStates[a].x = solution[a].states[timeStep].first.x;
    startStates[a].y = solution[a].states[timeStep].first.y;
    }
  }

  // for results only:
  int distance_to_new_goal = std::abs(startStates[agentNumber].x - goals[agentNumber].x ) + std::abs(startStates[agentNumber].y - goals[agentNumber].y );
  int distance_difference =  std::abs( distance_to_new_goal - distance_to_old_goal );
  myfile4 << distance_difference << std::endl;
  //if(approach == "baseline" ){
    // //make new environment and run cbs:
    Timer second_timer;
    Environment second_mapf(dimx, dimy, obstacles, goals);
    CBS<State, Action, int, Conflict, Constraints, Environment> second_cbs(second_mapf);
    std::vector<PlanResult<State, Action, int> > second_solution;

    
    bool second_success = second_cbs.search(startStates, second_solution, myfile2,agentNumber,timeStep,myfile3);
    myfile3 << std::endl;
    
    second_timer.stop();
    std::cout << second_timer.elapsedSeconds() <<std::endl;

      //Write second output file with the new paths
    WriteSolutionToOutputFile(second_success,second_solution,secondOutputFileBase,second_timer,&second_mapf);


  //}else{ // approach == "pruning"

    Timer second_tree_timer;
    //make new environment and run cbs:  
    Environment second_tree_mapf(dimx, dimy, obstacles, goals);
    TREE_CBS<State, Action, int, Conflict, Constraints, Environment> second_tree_cbs(second_tree_mapf);
    std::vector<PlanResult<State, Action, int> > second_tree_solution;

    //prun OCT
    int id = 1;
    Timer check_alg_time;
    std::vector<treeNode<TREE_CBS<State, Action, int, Conflict, Constraints, Environment>::HighLevelNode,Conflict>*> treeNodeVector = ct_tree->PreorderPrunTreeTraveling(agentNumber, timeStep,id, agentNumber, startStates);
    check_alg_time.stop();
    std::cout << "time to udate all nodes in new algorithem " << check_alg_time.elapsedSeconds() << std::endl;

    if (treeNodeVector.empty()){
      ct_tree->nullToRoot();
    }
    //debuge print:
    std::cout<< "new: number of relevent nodes to re-use:" << treeNodeVector.size() <<std::endl;
    myfile2 << treeNodeVector.size() << std:: endl;
    
    // run CBS with the newly created Open list

    //btree< TREE_CBS<State, Action, int, Conflict, Constraints, Environment>::HighLevelNode, Conflict> *second_ct_tree = new btree< TREE_CBS<State, Action, int, Conflict, Constraints, Environment>::HighLevelNode, Conflict>();

    
    bool second_tree_success = second_tree_cbs.search(startStates, second_tree_solution, ct_tree, treeNodeVector, id, agentNumber);
    second_tree_timer.stop();
    std::cout << second_tree_timer.elapsedSeconds() <<std::endl;

    WriteSolutionToOutputFile(second_tree_success,second_tree_solution,secondOutputFileNew,second_tree_timer,&second_tree_mapf);
  //}
  //************second phase- find new paths after "the agent" goal changed*************//
  
  // write results to file
  myfile << second_timer.elapsedSeconds() << "," << second_tree_timer.elapsedSeconds() << "\n";
  myfile.close();
  myfile2.close();
  myfile3.close();
  myfile4.close();



//*************old*************
  delete ct_tree;
  return 0;
}
