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

  // Print starts and goals
  std::cout << "[STARTS] ";
  for (uint i = 0; i < N; ++i)
    std::cout << i << "@" << ins->starts[i]->id << " ";
  std::cout << "\n";
  std::cout << "[GOALS]  ";
  for (uint i = 0; i < N; ++i)
    std::cout << i << "@" << ins->goals[i]->id << " ";
  std::cout << "\n";

  // Track agents that need to EXIT room
  // Only applies when goal is truly OUTSIDE (entrance corridor)
  // NOT when goal is in a tunnel between rooms
  std::vector<bool> needs_exit(N, false);
  for (uint ii = 0; ii < N; ++ii) {
    int sr = cell_to_room[ins->starts[ii]->id];
    int gr = cell_to_room[ins->goals[ii]->id];
    if (sr >= 0 && gr < 0) {
      // Goal is outside a room
      // Check if goal vertex has neighbours in 2+ different rooms
      // If yes = TUNNEL = don't boost
      // If no = ENTRANCE = boost!
      Vertex* gv = ins->goals[ii];
      std::set<int> adj_rooms;
      for (auto* nb : gv->neighbor) {
        int nr = cell_to_room[nb->id];
        if (nr >= 0) adj_rooms.insert(nr);
      }
      // Only entrance corridor if goal touches 0 or 1 room
      if (adj_rooms.size() <= 1)
        needs_exit[ii] = true;
    }
  }

  std::vector<Config> solution;
  solution.push_back(ins->starts);

  // Persistent PIBT priorities: +1 each step not at goal (anti-livelock),
  // reset to fractional part at goal. Survives across timesteps.
  std::vector<float> pri(N, 0.0f);
  bool pri_init = false;

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

    if (!pri_init) {
      for (uint pi = 0; pi < N; ++pi) pri[pi] = H->priorities[pi];
      pri_init = true;
    } else {
      for (uint pi = 0; pi < N; ++pi) {
        if (A[pi]->v_now != ins->goals[pi]) pri[pi] += 1.0f;
        else pri[pi] = pri[pi] - std::floor(pri[pi]);
      }
    }
    for (uint pi = 0; pi < N; ++pi) H->priorities[pi] = pri[pi];

    // ── PERSISTENT EXIT PRIORITY ─────────────────────────────────
    // Agent inside room with goal outside:
    // Keep boost until FULLY past entrance (in corridor)
    for (uint kk = 0; kk < N; ++kk) {
      if (needs_exit[kk]) {
        int cur = A[kk]->v_now->id;
        int cur_room = cell_to_room[cur];
        // Check if at entrance cell
        bool at_entrance = false;
        for (auto& room : rooms) {
          for (int e : room.entrances) {
            if (cur == e) { at_entrance = true; break; }
          }
          if (at_entrance) break;
        }
        // Clear only when fully in corridor (not at entrance)
        if (cur_room < 0 && !at_entrance)
          needs_exit[kk] = false;
        else
          H->priorities[kk] = 999.0f + H->priorities[kk];
      }
      // Room full → block entry
      if (approaching_full_room(A[kk])) {
        static int chits = 0; chits++;
        if (chits<=5) std::cout << "[COUNTER] agent " << kk << " step " << loop_cnt << "\n";
        H->priorities[kk] = 0.0001f;
      }
    }
    // Re-sort
    std::iota(H->order.begin(), H->order.end(), 0);
    std::sort(H->order.begin(), H->order.end(),
              [&](uint i, uint j) {
                return H->priorities[i] > H->priorities[j];
              });
    // ─────────────────────────────────────────────────────────────

    for (auto k : H->order) {
      auto* a = A[k];
      if (a->v_next != nullptr) continue;
      auto result = funcPIBT(a, a, H);
      // Mike's priority swap, persisted so it matters next timestep
      if (result.second != INT_MAX && result.second != -1) {
        std::swap(pri[k], pri[(uint)result.second]);
        std::swap(H->priorities[k], H->priorities[(uint)result.second]);
      }
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
  return ins->G.V[vid]->neighbor.size() <= 2;
}

void Planner::detect_rooms(int width, int height)
{
  cell_to_room.assign(V_size, -1);
  rooms.clear();

  auto deg = [&](int v){ return (int)ins->G.V[v]->neighbor.size(); };

  // ============================================================
  // EXACT ROOM DETECTION via single-cut test.
  // Definition: a "room" is a maximal region whose ONLY connection
  // to the rest of the map is through one corridor cell (a graph cut).
  // Regions with 2+ connections are open/through-space, not rooms.
  // ============================================================

  // STEP 1: classify every cell as WIDE (deg>=3) or THIN (deg<=2).
  // THIN cells are corridor candidates; WIDE cells are interior.

  // STEP 2: cluster WIDE cells into regions by flooding through WIDE cells
  // AND through THIN cells, but we will cut at thin "necks". To get exact
  // regions we instead flood the WHOLE free space but record, for each
  // region, the thin cells that act as its connectors.
  //
  // Practical exact method: for every THIN cell, test if it is a CUT
  // (its removal disconnects its neighbours). Collect cut cells. Then
  // remove all cut cells; the free space breaks into components; each
  // small component reachable through exactly ONE cut cell is a room.

  int N = (int)V_size;

  // --- find articulation: a thin cell is a "door" if removing it
  //     disconnects the local neighbourhood (its open neighbours can no
  //     longer reach each other without passing through it). ---
  auto reachable_without = [&](int a, int b, int blocked)->bool{
    if (a==blocked||b==blocked) return false;
    std::vector<char> seen(N,0);
    std::queue<int> q; q.push(a); seen[a]=1;
    while(!q.empty()){
      int c=q.front(); q.pop();
      if(c==b) return true;
      for(auto* nb:ins->G.V[c]->neighbor){
        int n=(int)nb->id;
        if(n==blocked||seen[n]) continue;
        seen[n]=1; q.push(n);
      }
    }
    return false;
  };

  // A thin cell is a DOOR if it has two open neighbours that cannot reach
  // each other when it is removed.
  std::vector<char> is_door(N,0);
  for(int c=0;c<N;c++){
    if(deg(c)>2) continue;             // doors are thin
    auto& nb=ins->G.V[c]->neighbor;
    bool door=false;
    for(size_t a=0;a<nb.size()&&!door;a++)
      for(size_t b=a+1;b<nb.size()&&!door;b++)
        if(!reachable_without((int)nb[a]->id,(int)nb[b]->id,c)) door=true;
    is_door[c]=door;
  }

  // STEP 3: remove all door cells; flood the remaining free space into
  // components. Each component's "exits" = the door cells adjacent to it.
  std::vector<int> comp(N,-1);
  int ncomp=0;
  for(int s=0;s<N;s++){
    if(is_door[s]||comp[s]>=0) continue;
    std::vector<int> cells; std::queue<int> q;
    q.push(s); comp[s]=ncomp;
    while(!q.empty()){
      int c=q.front(); q.pop(); cells.push_back(c);
      for(auto* nb:ins->G.V[c]->neighbor){
        int n=(int)nb->id;
        if(is_door[n]||comp[n]>=0) continue;
        comp[n]=ncomp; q.push(n);
      }
    }
    ncomp++;
  }

  // gather each component's cells and its set of adjacent doors
  std::vector<std::vector<int>> comp_cells(ncomp);
  std::vector<std::set<int>> comp_doors(ncomp);
  for(int c=0;c<N;c++){
    if(comp[c]<0) continue;
    comp_cells[comp[c]].push_back(c);
    for(auto* nb:ins->G.V[c]->neighbor)
      if(is_door[(int)nb->id]) comp_doors[comp[c]].insert((int)nb->id);
  }

  // STEP 4: classify. A component is a ROOM iff it connects to the rest
  // of the map through exactly ONE door. Report the exit distribution.
  std::map<int,int> exit_hist;
  int room_id=0, biggest=-1; size_t bigsz=0;
  for(int k=0;k<ncomp;k++){
    if(comp_cells[k].size()>bigsz){ bigsz=comp_cells[k].size(); biggest=k; }
  }
  for(int k=0;k<ncomp;k++){
    int exits=(int)comp_doors[k].size();
    exit_hist[exits]++;
    // the largest component is the open map; never a room
    if(k==biggest) continue;
    // a sealed component is only a ROOM if it has real interior (a wide cell).
    // all-thin components are hallway fragments, not rooms.
    bool has_interior=false;
    for(int c:comp_cells[k]) if(deg(c)>=3){ has_interior=true; break; }
    if(exits==1 && has_interior){
      RoomInfo room(room_id);
      for(int c:comp_cells[k]){ room.cells.push_back(c); cell_to_room[c]=room_id; }
      for(int d:comp_doors[k]) room.corridor_cells.push_back(d);
      room.entrances.push_back(*comp_doors[k].begin());
      room.capacity=(int)room.cells.size();
      room.current_count=0;
      rooms.push_back(room);
      room_id++;
    }
  }

  std::cout << "[STRUCT] components=" << ncomp << " exit distribution: ";
  for(auto& kv:exit_hist) std::cout << kv.second << "x(" << kv.first << "exit) ";
  std::cout << "\n";
  std::cout << "[ROOM] Detected " << rooms.size() << " sealed rooms (exactly 1 exit)\n";
  for(auto& r:rooms)
    std::cout << "[ROOM]   Room " << r.id << ": " << r.cells.size()
              << " cells, capacity=" << r.capacity
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
