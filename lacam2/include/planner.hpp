/*
 * lacam-star
 */

#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

// objective function
enum Objective { OBJ_NONE, OBJ_MAKESPAN, OBJ_SUM_OF_LOSS };
std::ostream& operator<<(std::ostream& os, const Objective objective);

// Room capacity tracking
struct RoomInfo {
  int id;
  std::vector<int> cells;           // cells on the enclosed side of the cut
  std::vector<int> corridor_cells;  // corridor cells that lead into this room
  std::vector<int> entrances;       // non-corridor endpoint cells of the cut
  int capacity;                     // max agents allowed in room+corridor
  int current_count;                // agents in room+corridor right now
  int num_doors;                    // number of entrance/door cells
  int room_counter;                 // vertex id: ROOM counter cell (near door)
  int rc_counter;                   // vertex id: RC counter cell (corridor mouth)
  int rc_capacity;                  // room + corridor cell count
  int rc_count;                     // agents in room + corridor right now
  int depth;                        // nesting depth (1=outermost)
  bool is_combined;                 // true = overlap counter, not a parent
  int parent;                       // id of containing room, -1 if root
  int outside;                      // 1 = giant/outside side, never gates entry
  RoomInfo(int _id) : id(_id), capacity(0), current_count(0), num_doors(0), room_counter(-1), rc_counter(-1), rc_capacity(0), rc_count(0), depth(1), is_combined(false), parent(-1), outside(0) {}
};

// PIBT agent
struct Agent {
  const uint id;
  Vertex* v_now;   // current location
  int root_agent; 
  Vertex* v_next;  // next location
  Agent(uint _id) : id(_id), v_now(nullptr), v_next(nullptr) {}
};
using Agents = std::vector<Agent*>;

// low-level node
struct LNode {
  std::vector<uint> who;
  Vertices where;
  const uint depth;
  LNode(LNode* parent = nullptr, uint i = 0,
        Vertex* v = nullptr);  // who and where
};

// high-level node
struct HNode {
  static uint HNODE_CNT;  // count #(high-level node)
  const Config C;

  // tree
  HNode* parent;
  std::set<HNode*> neighbor;

  // costs
  uint g;        // g-value (might be updated)
  const uint h;  // h-value
  uint f;        // g + h (might be updated)

  // for low-level search
  std::vector<float> priorities;
  std::vector<uint> order;
  std::queue<LNode*> search_tree;

  HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
        const uint _h);
  ~HNode();
};
using HNodes = std::vector<HNode*>;

struct Planner {
  const Instance* ins;
  const Deadline* deadline;
  std::mt19937* MT;
  const int verbose;

  // hyper parameters
  const Objective objective;
  const float RESTART_RATE;  // random restart

  // solver utils
  const uint N;       // number of agents
  const uint V_size;  // number o vertices
  DistTable D;
  uint loop_cnt;      // auxiliary

  // used in PIBT
  std::vector<std::array<Vertex*, 5> > C_next;  // next locations, used in PIBT
  std::vector<float> tie_breakers;              // random values, used in PIBT
  Agents A;
  Agents occupied_now;                          // for quick collision checking
  Agents occupied_next;                         // for quick collision checking

  Planner(const Instance* _ins, const Deadline* _deadline, std::mt19937* _MT,
          const int _verbose = 0,
          // other parameters
          const Objective _objective = OBJ_NONE,
          const float _restart_rate = 0.001);
  ~Planner();
  Solution solve(std::string& additional_info);
  void expand_lowlevel_tree(HNode* H, LNode* L);
  void rewrite(HNode* H_from, HNode* T, HNode* H_goal,
               std::stack<HNode*>& OPEN);
  uint get_edge_cost(const Config& C1, const Config& C2);
  uint get_edge_cost(HNode* H_from, HNode* H_to);
  uint get_h_value(const Config& C);
  bool get_new_config(HNode* H, LNode* L);
  std::pair<bool, int> funcPIBT(Agent* ai, Agent* root, HNode* H);

  // swap operation
  Agent* swap_possible_and_required(Agent* ai);
  bool is_swap_required(const uint pusher, const uint puller,
                        Vertex* v_pusher_origin, Vertex* v_puller_origin);
  bool is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin);

  // Room capacity system
  std::vector<RoomInfo> rooms;
  std::vector<int> cell_to_room;         // primary (innermost) room, -1 otherwise
  std::vector<std::vector<int>> cell_to_rooms;  // ALL rooms a cell belongs to (nesting/overlap)
  std::vector<int> corridor_to_room;     // corridor cells only, -1 otherwise
  std::vector<int> cell_to_region_room;  // room+corridor cells, -1 otherwise
  void detect_rooms(int width, int height);
  bool approaching_full_room(Agent* ai);
  int full_room_to_enter(Agent* ai);
  void update_room_counts();
  bool is_corridor(int vid) const;
  int entry_room_for_move(Agent* ai, Vertex* to) const;

  // utilities
  template <typename... Body>
  void solver_info(const int level, Body&&... body)
  {
    if (verbose < level) return;
    std::cout << "elapsed:" << std::setw(6) << elapsed_ms(deadline) << "ms"
              << "  loop_cnt:" << std::setw(8) << loop_cnt
              << "  node_cnt:" << std::setw(8) << HNode::HNODE_CNT << "\t";
    info(level, verbose, (body)...);
  }
};
