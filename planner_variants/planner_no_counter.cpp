#include "../include/planner.hpp"
#include <set>

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

HNode::HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g,
             const uint _h)
    : C(_C), parent(_parent), neighbor(), g(_g), h(_h), f(g + h),
      priorities(C.size()), order(C.size(), 0),
      search_tree(std::queue<LNode*>())
{
  ++HNODE_CNT;
  search_tree.push(new LNode());
  const auto N = C.size();
  if (parent != nullptr) parent->neighbor.insert(this);
  if (parent == nullptr) {
    for (uint i = 0; i < N; ++i) priorities[i] = (float)D.get(i, C[i]) / N;
  } else {
    for (size_t i = 0; i < N; ++i) {
      if (D.get(i, C[i]) != 0) priorities[i] = parent->priorities[i] + 1;
      else priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
    }
  }
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
    : ins(_ins), deadline(_deadline), MT(_MT), verbose(_verbose),
      objective(_objective), RESTART_RATE(_restart_rate),
      N(ins->N), V_size(ins->G.size()), D(DistTable(ins)), loop_cnt(0),
      C_next(N), tie_breakers(V_size, 0), A(N, nullptr),
      occupied_now(V_size, nullptr), occupied_next(V_size, nullptr)
{}

Planner::~Planner() {}

Solution Planner::solve(std::string& additional_info)
{
  solver_info(1, "start pure PIBT");

  for (auto i = 0; i < N; ++i) {
    A[i] = new Agent(i);
    A[i]->v_now  = ins->starts[i];
    A[i]->v_next = nullptr;
    occupied_now[ins->starts[i]->id] = A[i];
  }

  detect_rooms(ins->G.width, ins->G.height);

  std::vector<Config> solution;
  solution.push_back(ins->starts);

  while (!is_expired(deadline)) {
    loop_cnt++;

    bool all_done = true;
    for (uint i = 0; i < N; ++i) {
      if (A[i]->v_now != ins->goals[i]) { all_done = false; break; }
    }
    if (all_done) break;

    for (auto* a : A) {
      if (a->v_next != nullptr) {
        occupied_next[a->v_next->id] = nullptr;
        a->v_next = nullptr;
      }
      a->root_agent = -1;
    }

    update_room_counts();

    Config C_now(N);
    for (uint i = 0; i < N; ++i) C_now[i] = A[i]->v_now;

    auto H = new HNode(C_now, D, nullptr,
                       (uint)solution.size(),
                       get_h_value(C_now));

    // ── ROOM CAPACITY CHECK DISABLED ───────────────────────────

    for (auto k : H->order) {
      auto* a = A[k];
      if (a->v_next != nullptr) continue;
      funcPIBT(a, a, H);
    }

    Config C_new(N);
    for (auto* a : A) {
      if (a->v_next == nullptr) a->v_next = a->v_now;
      C_new[a->id] = a->v_next;
    }

    if (C_new == solution.back()) {
      solver_info(1, "deadlock at step ", loop_cnt);
      break;
    }

    // Show first 25 steps
    if (loop_cnt <= 25) {
      std::cout << "[T=" << loop_cnt << "] ";
      for (uint i = 0; i < N; ++i) {
        int rid = cell_to_room[A[i]->v_now->id];
        std::string loc = (rid >= 0) ? "IN" : "OUT";
        std::cout << i << "@" << A[i]->v_now->id
                  << "(" << loc << ") ";
      }
      std::cout << "\n";
    }

    solution.push_back(C_new);

    // FIX: clear ALL old cells first, THEN occupy new ones.
    // Interleaved clear/set corrupted occupied_now for chained moves.
    for (auto* a : A) occupied_now[a->v_now->id] = nullptr;
    for (auto* a : A) {
      a->v_now = a->v_next;
      occupied_now[a->v_now->id] = a;
    }

    delete H;
  }

  solver_info(1, "done in ", loop_cnt, " steps");
  additional_info += "mode=pure_pibt\n";
  additional_info += "steps=" + std::to_string(loop_cnt) + "\n";
  for (auto* a : A) delete a;
  return solution;
}

void Planner::rewrite(HNode* H_from, HNode* H_to, HNode* H_goal,
                      std::stack<HNode*>& OPEN)
{
  H_from->neighbor.insert(H_to);
  std::queue<HNode*> Q({H_from});
  while (!Q.empty()) {
    auto n_from = Q.front(); Q.pop();
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
    for (uint i = 0; i < N; ++i)
      if (C1[i] != ins->goals[i] || C2[i] != ins->goals[i]) cost += 1;
    return cost;
  }
  return 1;
}

uint Planner::get_edge_cost(HNode* H_from, HNode* H_to)
{
  return get_edge_cost(H_from->C, H_to->C);
}

uint Planner::get_h_value(const Config& C)
{
  uint cost = 0;
  if (objective == OBJ_MAKESPAN)
    for (auto i = 0; i < N; ++i) cost = std::max(cost, D.get(i, C[i]));
  else if (objective == OBJ_SUM_OF_LOSS)
    for (auto i = 0; i < N; ++i) cost += D.get(i, C[i]);
  return cost;
}

void Planner::expand_lowlevel_tree(HNode* H, LNode* L)
{
  if (L->depth >= N) return;
  const auto i = H->order[L->depth];
  auto C = H->C[i]->neighbor;
  C.push_back(H->C[i]);
  if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT);
  for (auto v : C) H->search_tree.push(new LNode(L, i, v));
}

bool Planner::get_new_config(HNode* H, LNode* L)
{
  for (auto a : A) {
    if (a->v_now != nullptr && occupied_now[a->v_now->id] == a)
      occupied_now[a->v_now->id] = nullptr;
    if (a->v_next != nullptr) {
      occupied_next[a->v_next->id] = nullptr;
      a->v_next = nullptr;
    }
    a->root_agent = -1;
    a->v_now = H->C[a->id];
    occupied_now[a->v_now->id] = a;
  }
  for (uint k = 0; k < L->depth; ++k) {
    const auto i = L->who[k];
    const auto l = L->where[k]->id;
    if (occupied_next[l] != nullptr) return false;
    auto l_pre = H->C[i]->id;
    if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
        occupied_next[l_pre]->id == occupied_now[l]->id)
      return false;
    A[i]->v_next = L->where[k];
    occupied_next[l] = A[i];
  }
  update_room_counts();
  for (auto k : H->order) {
    auto a = A[k];
    if (a->v_next == nullptr) {
      auto result = funcPIBT(a, a, H);
      if (result.second != INT_MAX && result.second != -1) {
        auto temp = H->priorities[k];
        H->priorities[k] = H->priorities[result.second];
        H->priorities[result.second] = temp;
      }
      if (!result.first) return false;
    }
  }
  return true;
}

bool Planner::is_corridor(int vid) const
{
  if (vid < 0 || vid >= (int)V_size) return false;
  Vertex* v = nullptr;
  for (auto* u : ins->G.V) {
    if ((int)u->id == vid) { v = u; break; }
  }
  if (!v) return false;
  return v->neighbor.size() == 2;
}

void Planner::detect_rooms(int width, int height)
{
  const int W = ins->G.width;
  const int H = ins->G.height;
  const int total = W * H;

  cell_to_room.assign(V_size, -1);

  std::vector<bool> wall_row(H, false);
  std::vector<bool> wall_col(W, false);

  for (int r = 0; r < H; ++r) {
    int wc = 0;
    for (int c = 0; c < W; ++c) {
      int idx = r * W + c;
      if (idx >= total || ins->G.U[idx] == nullptr) wc++;
    }
    if ((float)wc / W > 0.5f) wall_row[r] = true;
  }
  for (int c = 0; c < W; ++c) {
    int wc = 0;
    for (int r = 0; r < H; ++r) {
      int idx = r * W + c;
      if (idx >= total || ins->G.U[idx] == nullptr) wc++;
    }
    if ((float)wc / H > 0.5f) wall_col[c] = true;
  }

  std::vector<int> row_bounds, col_bounds;
  for (int r = 0; r < H; ++r) if (wall_row[r]) row_bounds.push_back(r);
  for (int c = 0; c < W; ++c) if (wall_col[c]) col_bounds.push_back(c);

  int room_id = 0;
  if (row_bounds.size() >= 2 && col_bounds.size() >= 2) {
    for (int ri = 0; ri + 1 < (int)row_bounds.size(); ++ri) {
      for (int ci = 0; ci + 1 < (int)col_bounds.size(); ++ci) {
        int r0 = row_bounds[ri]+1, r1 = row_bounds[ri+1]-1;
        int c0 = col_bounds[ci]+1, c1 = col_bounds[ci+1]-1;
        if (r0 > r1 || c0 > c1) continue;

        RoomInfo room(room_id);
        for (int r = r0; r <= r1; ++r)
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
        if (room.cells.empty()) continue;

        auto add_ent = [&](int r, int c) {
          int idx = r * W + c;
          if (idx >= 0 && idx < total && ins->G.U[idx] != nullptr) {
            int vid = ins->G.U[idx]->id;
            if (vid < (int)V_size) room.entrances.push_back(vid);
          }
        };
        for (int c = c0; c <= c1; ++c) add_ent(row_bounds[ri], c);
        for (int c = c0; c <= c1; ++c) add_ent(row_bounds[ri+1], c);
        for (int r = r0; r <= r1; ++r) add_ent(r, col_bounds[ci]);
        for (int r = r0; r <= r1; ++r) add_ent(r, col_bounds[ci+1]);

        room.capacity = std::max(1, (int)room.cells.size() / 2);
        room.current_count = 0;
        rooms.push_back(room);
        room_id++;
      }
    }
  }

  // Boundary-aware corridor flood fill
  std::vector<bool> visited(V_size, false);
  for (int i = 0; i < (int)V_size; i++)
    if (cell_to_room[i] >= 0) visited[i] = true;

  for (auto* v : ins->G.V) {
    if (!is_corridor((int)v->id)) continue;
    if (visited[v->id]) continue;

    std::queue<Vertex*> q;
    std::vector<int> comp;
    q.push(v);
    visited[v->id] = true;

    std::set<int> connected_rooms;
    bool hits_open = false;

    while (!q.empty()) {
      auto* cur = q.front(); q.pop();
      comp.push_back(cur->id);
      for (auto* nb : cur->neighbor) {
        int nb_room = cell_to_room[nb->id];
        if (nb_room >= 0) { connected_rooms.insert(nb_room); continue; }
        if (visited[nb->id]) continue;
        visited[nb->id] = true;
        if ((int)nb->neighbor.size() >= 3) hits_open = true;
        q.push(nb);
      }
    }

    std::string ctype = "HALLWAY";
    if (connected_rooms.size() == 1 && hits_open)  ctype = "ENTRANCE";
    if (connected_rooms.size() >= 2)               ctype = "TUNNEL";
    if (connected_rooms.size() == 1 && !hits_open) ctype = "ALCOVE";

    std::cout << "[CORR] " << ctype << ": " << comp.size()
              << " cells, rooms=[";
    for (int r : connected_rooms) std::cout << r << " ";
    std::cout << "]\n";
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
  for (auto& r : rooms) r.current_count = 0;
  for (auto* a : A) {
    if (a->v_now == nullptr) continue;
    int rid = cell_to_room[a->v_now->id];
    if (rid >= 0 && rid < (int)rooms.size())
      rooms[rid].current_count++;
  }
}

bool Planner::approaching_full_room(Agent* ai)
{
  if (rooms.empty() || ai->v_now == nullptr) return false;
  int current_room = cell_to_room[ai->v_now->id];
  if (current_room >= 0) return false;
  for (auto* nb : ai->v_now->neighbor) {
    int rid = cell_to_room[nb->id];
    if (rid < 0) continue;
    if (rooms[rid].current_count >= rooms[rid].capacity) return true;
  }
  return false;
}

std::pair<bool, int> Planner::funcPIBT(Agent* ai, Agent* root, HNode* H)
{
  const auto i = ai->id;
  const auto K = ai->v_now->neighbor.size();
  ai->root_agent = root->id;

  for (auto k = 0; k < K; ++k) {
    auto u = ai->v_now->neighbor[k];
    C_next[i][k] = u;
    if (MT != nullptr) tie_breakers[u->id] = get_random_float(MT);
  }
  C_next[i][K] = ai->v_now;

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

  for (auto k = 0; k < K + 1; ++k) {
    auto u = C_next[i][k];
    if (occupied_next[u->id] != nullptr) {
      if (H->priorities[occupied_next[u->id]->id] > H->priorities[root->id])
        pswap_agent = INT_MAX;
      else if (pswap_agent == -1 ||
               (pswap_agent != INT_MAX &&
                H->priorities[occupied_next[u->id]->id] > H->priorities[pswap_agent] &&
                H->priorities[occupied_next[u->id]->id] < H->priorities[root->id]))
        pswap_agent = occupied_next[u->id]->id;
      continue;
    }
    auto& ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == ai->v_now) continue;
    occupied_next[u->id] = ai;
    ai->v_next = u;
    if (ak != nullptr && ak != ai && ak->v_next == nullptr) {
      auto result = funcPIBT(ak, root, H);
      if (!result.first) {
        if (result.second == INT_MAX) pswap_agent = INT_MAX;
        else if (pswap_agent == -1 ||
                 (pswap_agent != INT_MAX &&
                  H->priorities[result.second] > H->priorities[pswap_agent]))
          pswap_agent = result.second;
        continue;
      }
    }
    if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
        occupied_next[ai->v_now->id] == nullptr) {
      swap_agent->v_next = ai->v_now;
      occupied_next[swap_agent->v_next->id] = swap_agent;
    }
    if (ai == root && ai->v_next == ai->v_now)
      return std::make_pair(true, pswap_agent);
    return std::make_pair(true, INT_MAX);
  }

  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return std::make_pair(false, pswap_agent);
}

Agent* Planner::swap_possible_and_required(Agent* ai)
{
  const auto i = ai->id;
  if (C_next[i][0] == ai->v_now) return nullptr;
  auto aj = occupied_now[C_next[i][0]->id];
  if (aj != nullptr && aj->v_next == nullptr &&
      is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
      is_swap_possible(aj->v_now, ai->v_now))
    return aj;
  for (auto u : ai->v_now->neighbor) {
    auto ak = occupied_now[u->id];
    if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
    if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0]) &&
        is_swap_possible(C_next[i][0], ai->v_now))
      return ak;
  }
  return nullptr;
}

bool Planner::is_swap_required(const uint pusher, const uint puller,
                               Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (D.get(pusher, v_puller) < D.get(pusher, v_pusher)) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u))
        --n;
      else tmp = u;
    }
    if (n >= 2) return false;
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return (D.get(puller, v_pusher) < D.get(puller, v_puller)) &&
         (D.get(pusher, v_pusher) == 0 ||
          D.get(pusher, v_puller) < D.get(pusher, v_pusher));
}

bool Planner::is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex* tmp = nullptr;
  while (v_puller != v_pusher_origin) {
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      auto a = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u))
        --n;
      else tmp = u;
    }
    if (n >= 2) return true;
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}

std::ostream& operator<<(std::ostream& os, const Objective obj)
{
  if (obj == OBJ_NONE) os << "none";
  else if (obj == OBJ_MAKESPAN) os << "makespan";
  else if (obj == OBJ_SUM_OF_LOSS) os << "sum_of_loss";
  return os;
}
