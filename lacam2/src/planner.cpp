#include "../include/planner.hpp"

LNode::LNode(LNode* parent, uint i, Vertex* v)
    : who(), where(), depth(parent == nullptr ? 0 : parent->depth + 1)
{
  if (parent != nullptr) {
    who = parent->who;
    who.push_back(i);
    where = parent->where;
    where.push_back(v);
  }
}

uint HNode::HNODE_CNT = 0;

// for high-level
HNode::HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
             const uint _h)
    : C(_C),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      f(g + h),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<LNode*>())
{
  ++HNODE_CNT;

  search_tree.push(new LNode());
  const auto N = C.size();

  // update neighbor
  if (parent != nullptr) parent->neighbor.insert(this);

  // set priorities
  if (parent == nullptr) {
    // initialize
    for (uint i = 0; i < N; ++i) priorities[i] = (float)D.get(i, C[i]) / N;
  } else {
    // dynamic priorities, akin to PIBT
    for (size_t i = 0; i < N; ++i) {
      if (D.get(i, C[i]) != 0) {
        priorities[i] = parent->priorities[i] + 1;
      } else {
        priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
      }
    }
  }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](uint i, uint j) { return priorities[i] > priorities[j]; });
}

HNode::~HNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(const Instance* _ins, const Deadline* _deadline,
                 std::mt19937* _MT, const int _verbose,
                 const Objective _objective, const float _restart_rate)
    : ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      objective(_objective),
      RESTART_RATE(_restart_rate),
      N(ins->N),
      V_size(ins->G.size()),
      D(DistTable(ins)),
      loop_cnt(0),
      C_next(N),
      tie_breakers(V_size, 0),
      A(N, nullptr),
      occupied_now(V_size, nullptr),
      occupied_next(V_size, nullptr)
{
}

Planner::~Planner() {}

Solution Planner::solve(std::string& additional_info)
{
  solver_info(1, "start search");

  // setup agents
  for (auto i = 0; i < N; ++i) A[i] = new Agent(i);
  // setup room capacity system
  detect_rooms(ins->G.width, ins->G.height);

  // setup search
  auto OPEN = std::stack<HNode*>();
  auto EXPLORED = std::unordered_map<Config, HNode*, ConfigHasher>();
  // insert initial node, 'H': high-level node
  auto H_init = new HNode(ins->starts, D, nullptr, 0, get_h_value(ins->starts));
  OPEN.push(H_init);
  EXPLORED[H_init->C] = H_init;

  std::vector<Config> solution;
  auto C_new = Config(N, nullptr);  // for new configuration
  HNode* H_goal = nullptr;          // to store goal node

  // DFS
  while (!OPEN.empty() && !is_expired(deadline)) {
    loop_cnt += 1;

    // do not pop here!
    auto H = OPEN.top();  // high-level node

    // low-level search end
    if (H->search_tree.empty()) {
      OPEN.pop();
      continue;
    }

    // check lower bounds
    if (H_goal != nullptr && H->f >= H_goal->f) {
      OPEN.pop();
      continue;
    }

    // check goal condition
    if (H_goal == nullptr && is_same_config(H->C, ins->goals)) {
      H_goal = H;
      solver_info(1, "found solution, cost: ", H->g);
      if (objective == OBJ_NONE) break;
      continue;
    }

    // create successors at the low-level search
    auto L = H->search_tree.front();
    H->search_tree.pop();
    expand_lowlevel_tree(H, L);

    // create successors at the high-level search
    const auto res = get_new_config(H, L);
    delete L;  // free
    if (!res) std::cout << "failed to get new config" << std::endl;

    // create new configuration
    for (auto a : A) C_new[a->id] = a->v_next;

    if (C_new == H->C) {
      std::cout<< "dead lock" << std::endl;
    }

    // check explored list
    // const auto iter = EXPLORED.find(C_new);
    // if (iter != EXPLORED.end()) {
    //   // case found
    //   rewrite(H, iter->second, H_goal, OPEN);
    //   // re-insert or random-restart
    //   auto H_insert = (MT != nullptr && get_random_float(MT) >= RESTART_RATE)
    //                       ? iter->second
    //                       : H_init;
    //   if (H_goal == nullptr || H_insert->f < H_goal->f) OPEN.push(H_insert);
    // } else {
      // insert new search node
      const auto H_new = new HNode(
          C_new, D, H, H->g + get_edge_cost(H->C, C_new), get_h_value(C_new));
      EXPLORED[H_new->C] = H_new;
      if (H_goal == nullptr || H_new->f < H_goal->f) OPEN.push(H_new);
    // }
  }

  // backtrack
  if (H_goal != nullptr) {
    auto H = H_goal;
    while (H != nullptr) {
      solution.push_back(H->C);
      H = H->parent;
    }
    std::reverse(solution.begin(), solution.end());
  }

  // print result
  if (H_goal != nullptr && OPEN.empty()) {
    solver_info(1, "solved optimally, objective: ", objective);
  } else if (H_goal != nullptr) {
    solver_info(1, "solved sub-optimally, objective: ", objective);
  } else if (OPEN.empty()) {
    solver_info(1, "no solution");
  } else {
    solver_info(1, "timeout");
  }

  // logging
  additional_info +=
      "optimal=" + std::to_string(H_goal != nullptr && OPEN.empty()) + "\n";
  additional_info += "objective=" + std::to_string(objective) + "\n";
  additional_info += "loop_cnt=" + std::to_string(loop_cnt) + "\n";
  additional_info += "num_node_gen=" + std::to_string(EXPLORED.size()) + "\n";

  // memory management
  for (auto a : A) delete a;
  for (auto itr : EXPLORED) delete itr.second;

  return solution;
}

void Planner::rewrite(HNode* H_from, HNode* H_to, HNode* H_goal,
                      std::stack<HNode*>& OPEN)
{
  // update neighbors
  H_from->neighbor.insert(H_to);

  // Dijkstra update
  std::queue<HNode*> Q({H_from});  // queue is sufficient
  while (!Q.empty()) {
    auto n_from = Q.front();
    Q.pop();
    for (auto n_to : n_from->neighbor) {
      auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
      if (g_val < n_to->g) {
        if (n_to == H_goal)
          solver_info(1, "cost update: ", n_to->g, " -> ", g_val);
        n_to->g = g_val;
        n_to->f = n_to->g + n_to->h;
        n_to->parent = n_from;
        Q.push(n_to);
        if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push(n_to);
      }
    }
  }
}

uint Planner::get_edge_cost(const Config& C1, const Config& C2)
{
  if (objective == OBJ_SUM_OF_LOSS) {
    uint cost = 0;
    for (uint i = 0; i < N; ++i) {
      if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) {
        cost += 1;
      }
    }
    return cost;
  }

  // default: makespan
  return 1;
}

uint Planner::get_edge_cost(HNode* H_from, HNode* H_to)
{
  return get_edge_cost(H_from->C, H_to->C);
}

uint Planner::get_h_value(const Config& C)
{
  uint cost = 0;
  if (objective == OBJ_MAKESPAN) {
    for (auto i = 0; i < N; ++i) cost = std::max(cost, D.get(i, C[i]));
  } else if (objective == OBJ_SUM_OF_LOSS) {
    for (auto i = 0; i < N; ++i) cost += D.get(i, C[i]);
  }
  return cost;
}

void Planner::expand_lowlevel_tree(HNode* H, LNode* L)
{
  if (L->depth >= N) return;
  const auto i = H->order[L->depth];
  auto C = H->C[i]->neighbor;
  C.push_back(H->C[i]);
  // randomize
  if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
  // insert
  for (auto v : C) H->search_tree.push(new LNode(L, i, v));
}

bool Planner::get_new_config(HNode* H, LNode* L)
{
  // setup cache
  for (auto a : A) {
    // clear previous cache
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) {
      occupied_now[a->v_now->id] = nullptr;
    }
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }
    a->root_agent = -1;  // reset root agent

    // set occupied now
    a->v_now = H->C[a->id];
    occupied_now[a->v_now->id] = a;
  }

  // add constraints
  for (uint k = 0; k < L->depth; ++k) {
    const auto i = L->who[k];        // agent
    const auto l = L->where[k]->id;  // loc

    // check vertex collision
    if (occupied_next[l] != nullptr) return false;
    // check swap collision
    auto l_pre = H->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
      return false;

    // set occupied_next
    A[i]->v_next = L->where[k];
    occupied_next[l] = A[i];
  }

  // update room counts before PIBT runs
  update_room_counts();

  // perform PIBT
  for (auto k : H->order) {
    auto a = A[k];
    if (a->v_next == nullptr){ 
      auto result = funcPIBT(a, a, H);
      if (result.second != INT_MAX && result.second != -1) {
        // std::cout<< "swap: " << a->id <<":"<< H->priorities[a->id] << " with " << result.second <<":"<< H->priorities[result.second] << std::endl;
        // priority swap
        auto temp = H->priorities[k];
        H->priorities[k] =  H->priorities[result.second];  
        H->priorities[result.second] = temp;
      }
      if (!result.first) return false;  // planning failure
    }
  }
  return true;
}

// ── Room Capacity System ─────────────────────────────────────────────────

void Planner::detect_rooms(int width, int height)
{
  // Use graph dimensions directly
  const int W = ins->G.width;
  const int H = ins->G.height;
  const int total = W * H;

  std::cout << "[DEBUG] W=" << W << " H=" << H
            << " total=" << total
            << " V_size=" << V_size
            << " U.size=" << ins->G.U.size() << "\n";

  // Initialize cell_to_room with -1 (no room)
  cell_to_room.assign(V_size, -1);
  std::cout << "[DEBUG] cell_to_room assigned\n";

  // For each row: count walls
  std::vector<bool> wall_row(H, false);
  std::vector<bool> wall_col(W, false);

  for (int r = 0; r < H; ++r) {
    int wall_count = 0;
    for (int c = 0; c < W; ++c) {
      int idx = r * W + c;
      if (idx >= total || ins->G.U[idx] == nullptr) wall_count++;
    }
    if ((float)wall_count / W > 0.5f) wall_row[r] = true;
  }

  for (int c = 0; c < W; ++c) {
    int wall_count = 0;
    for (int r = 0; r < H; ++r) {
      int idx = r * W + c;
      if (idx >= total || ins->G.U[idx] == nullptr) wall_count++;
    }
    if ((float)wall_count / H > 0.5f) wall_col[c] = true;
  }

  std::vector<int> row_bounds, col_bounds;
  for (int r = 0; r < H; ++r) if (wall_row[r]) row_bounds.push_back(r);
  for (int c = 0; c < W; ++c) if (wall_col[c]) col_bounds.push_back(c);

  if (row_bounds.size() < 2 || col_bounds.size() < 2) {
    std::cout << "[ROOM] No room structure detected\n";
    return;
  }

  int room_id = 0;
  for (int ri = 0; ri + 1 < (int)row_bounds.size(); ++ri) {
    for (int ci = 0; ci + 1 < (int)col_bounds.size(); ++ci) {
      int r0 = row_bounds[ri]     + 1;
      int r1 = row_bounds[ri + 1] - 1;
      int c0 = col_bounds[ci]     + 1;
      int c1 = col_bounds[ci + 1] - 1;

      if (r0 > r1 || c0 > c1) continue;

      RoomInfo room(room_id);

      // Interior cells
      for (int r = r0; r <= r1; ++r) {
        for (int c = c0; c <= c1; ++c) {
          int idx = r * W + c;
          if (idx >= 0 && idx < total && ins->G.U[idx] != nullptr) {
            int vid = ins->G.U[idx]->id;
            if (vid < (int)V_size) {
              room.cells.push_back(vid);
              cell_to_room[vid] = room_id;
            }
          }
        }
      }

      if (room.cells.empty()) continue;

      // Entrance cells: gaps in boundary walls adjacent to this room
      // Top boundary row
      for (int c = c0; c <= c1; ++c) {
        int idx = row_bounds[ri] * W + c;
        if (idx >= 0 && idx < total && ins->G.U[idx] != nullptr) {
          int vid = ins->G.U[idx]->id;
          if (vid < (int)V_size) room.entrances.push_back(vid);
        }
      }
      // Bottom boundary row
      for (int c = c0; c <= c1; ++c) {
        int idx = row_bounds[ri + 1] * W + c;
        if (idx >= 0 && idx < total && ins->G.U[idx] != nullptr) {
          int vid = ins->G.U[idx]->id;
          if (vid < (int)V_size) room.entrances.push_back(vid);
        }
      }
      // Left boundary col
      for (int r = r0; r <= r1; ++r) {
        int idx = r * W + col_bounds[ci];
        if (idx >= 0 && idx < total && ins->G.U[idx] != nullptr) {
          int vid = ins->G.U[idx]->id;
          if (vid < (int)V_size) room.entrances.push_back(vid);
        }
      }
      // Right boundary col
      for (int r = r0; r <= r1; ++r) {
        int idx = r * W + col_bounds[ci + 1];
        if (idx >= 0 && idx < total && ins->G.U[idx] != nullptr) {
          int vid = ins->G.U[idx]->id;
          if (vid < (int)V_size) room.entrances.push_back(vid);
        }
      }

      // capacity = half room size (realistic threshold)
      room.capacity = std::max(1, (int)room.cells.size() / 2);
      room.current_count = 0;
      rooms.push_back(room);
      room_id++;
    }
  }

  std::cout << "[ROOM] Detected " << rooms.size() << " rooms\n";
  for (auto& r : rooms)
    std::cout << "[ROOM]   Room " << r.id
              << ": " << r.cells.size() << " cells"
              << ", capacity=" << r.capacity
              << ", entrances=" << r.entrances.size() << "\n";
}

void Planner::update_room_counts()
{
  // Reset all counts
  for (auto& r : rooms) r.current_count = 0;

  // Count agents in each room
  for (auto* a : A) {
    if (a->v_now == nullptr) continue;
    int rid = cell_to_room[a->v_now->id];
    if (rid >= 0 && rid < (int)rooms.size())
      rooms[rid].current_count++;
  }
}

bool Planner::approaching_full_room(Agent* ai)
{
  if (rooms.empty()) return false;
  if (ai->v_now == nullptr) return false;

  int current_room = cell_to_room[ai->v_now->id];

  // Check each neighbour the agent might move to
  for (auto* nb : ai->v_now->neighbor) {
    int rid = cell_to_room[nb->id];
    if (rid < 0) continue;           // neighbour not in any room
    if (rid == current_room) continue; // already in same room

    // Agent is outside this room, neighbour is inside
    auto& room = rooms[rid];

    static int debug_count = 0;
    debug_count++;
    if (debug_count <= 20) {
      std::cout << "[DBG] Agent " << ai->id 
                << " at " << ai->v_now->id
                << " room=" << current_room
                << " -> nb=" << nb->id
                << " rid=" << rid
                << " count=" << room.current_count
                << " cap=" << room.capacity
                << " entrances=" << room.entrances.size()
                << "\n";
    }

    // Is room full?
    if (room.current_count >= room.capacity) {
      return true;
    }
  }
  return false;
}

// ── End Room Capacity System ──────────────────────────────────────────────

std::pair<bool, int> Planner::funcPIBT(Agent* ai, Agent* root, HNode* H)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();
  ai->root_agent = root->id;  // set root agent

  // CAPACITY CHECK: soft version
  // Don't freeze agent, just lower priority
  // so it prefers NOT entering full rooms
  if (ai == root && approaching_full_room(ai)) {
    // Reduce priority but don't freeze
    // Agent can still enter if it must
    H->priorities[i] = H->priorities[i] * 0.1f;
  }

  // get candidates for next locations
  for (auto k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr)
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
  }
  C_next[i][K] = ai->v_now;

  // sort
  std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
            [&](Vertex* const v, Vertex* const u) {
              return D.get(i, v) + tie_breakers[v->id] <
                     D.get(i, u) + tie_breakers[u->id];
            });

  Agent* swap_agent = swap_possible_and_required(ai);
  if (swap_agent != nullptr)
    std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);

  int pswap_agent = -1;
  if (root != ai) pswap_agent = i;

  // main operation
  for (auto k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];

    // avoid vertex conflicts
    if (occupied_next[u->id] != nullptr) {
      if (H->priorities[occupied_next[u->id]->id] > H->priorities[root->id])
        pswap_agent = INT_MAX;
      else if(pswap_agent == -1 || 
        (pswap_agent != INT_MAX && 
          H->priorities[occupied_next[u->id]->id] > H->priorities[pswap_agent] && 
          H->priorities[occupied_next[u->id]->id] < H->priorities[root->id])
        )
        pswap_agent = occupied_next[u->id]->id;
      continue;
    }

    auto& ak = occupied_now[u->id];

    // avoid swap conflicts
    if (ak != nullptr && ak->v_next == ai->v_now) continue;

    // reserve next location
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // priority inheritance
    if (ak != nullptr && ak != ai && ak->v_next == nullptr){
        auto result =  funcPIBT(ak, root, H);
      if (!result.first){
        if (result.second == INT_MAX) {
          pswap_agent = INT_MAX;  // no swap possible
        }
        else if (pswap_agent == -1 || (pswap_agent != INT_MAX && H->priorities[result.second] > H->priorities[pswap_agent])) {
          pswap_agent = result.second;  // update pswap_agent
        }
        continue;
      }
    }

    // success to plan next one step
    // pull swap_agent when applicable
    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      swap_agent->v_next = ai->v_now;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }

    if (ai == root && ai->v_next == ai->v_now){
      return std::make_pair(true, pswap_agent);  // root agent is done
    }
    return std::make_pair(true, INT_MAX);
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return std::make_pair(false, pswap_agent);
}

Agent* Planner::swap_possible_and_required(Agent* ai)
{
  const auto i = ai->id;
  // ai wanna stay at v_now -> no need to swap
  if (C_next[i][0] == ai->v_now) return nullptr;

  // usual swap situation, c.f., case-a, b
  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
      is_swap_possible(aj->v_now, ai->v_now)) {
    return aj;
  }

  // for clear operation, c.f., case-c
  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0]) &&
        is_swap_possible(C_next[i][0], ai->v_now)) {
      return ak;
    }
  }

  return nullptr;
}

// simulate whether the swap is required
bool Planner::is_swap_required(const uint pusher, const uint puller,
                               Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (D.get(pusher, v_puller) < D.get(pusher, v_pusher)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  // judge based on distance
  return (D.get(puller, v_pusher) < D.get(puller, v_puller)) &&
         (D.get(pusher, v_pusher) == 0 ||
          D.get(pusher, v_puller) < D.get(pusher, v_pusher));
}

// simulate whether the swap is possible
bool Planner::is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (v_puller != v_pusher_origin) {  // avoid loop
    auto n = v_puller->neighbor.size();  // count #(possible locations) to pull
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) {
        --n;      // pull-impossible with u
      } else {
        tmp = u;  // pull-possible with u
      }
    }
    if (n >= 2) return true;  // able to swap
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

std::ostream& operator<<(std::ostream& os, const Objective obj)
{
  if (obj == OBJ_NONE) {
    os << "none";
  } else if (obj == OBJ_MAKESPAN) {
    os << "makespan";
  } else if (obj == OBJ_SUM_OF_LOSS) {
    os << "sum_of_loss";
  }
  return os;
}
