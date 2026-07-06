#include "../include/planner.hpp"
#include <set>
#include <array>
static int g_fire_count = 0;
static int g_fire_agent = -1;
static int g_frozen_cap = 60;
static int g_exit_boost = 0;
static int g_park_timeout = 50;  // release a parked agent if its room won't drain within this many steps  // OFF by default (old experiment; caused swap ping-pong)  // tunable: max frozen steps to wait while an agent is parked

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
  std::vector<int> waiting_room(N, -1);
  std::vector<int> park_duration(N, 0);
  int frozen_count = 0;
  { const char* fc=getenv("FROZEN_CAP"); if(fc) g_frozen_cap=atoi(fc); }
  { const char* eb=getenv("EXIT_BOOST"); if(eb) g_exit_boost=atoi(eb); }
  { const char* pt=getenv("PARK_TIMEOUT"); if(pt) g_park_timeout=atoi(pt); }
  std::vector<float> orig_pri(N, 0.0f);

  while (!is_expired(deadline)) {
    loop_cnt++;
    if (loop_cnt > 2000) break;   // uniform step cap (all builds): >2000 steps = degenerate = failure

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
#if defined(USE_ANDY) && !defined(NO_COUNTER)
    if (loop_cnt >= 12 && loop_cnt <= 45 && !rooms.empty()) {
      std::cout << "\n=== STEP " << loop_cnt << " | room0=" << rooms[0].current_count
                << "/" << rooms[0].capacity << " ===\n  IN ROOM0: ";
      for (uint i=0;i<N;i++){
        bool inR=false; for(int rid:cell_to_rooms[A[i]->v_now->id]) if(rid==0) inR=true;
        if(inR){ int gr=cell_to_room[ins->goals[i]->id];
          std::cout<<"a"<<i<<"(pri"<<pri[i]<<",goal"<<(gr==0?"IN":"OUT")<<") "; }
      }
      std::cout << "\n  PARKED: ";
      for (uint i=0;i<N;i++) if(waiting_room[i]>=0)
        std::cout<<"a"<<i<<"(wants"<<waiting_room[i]<<",opri"<<orig_pri[i]<<",at"<<A[i]->v_now->id<<") ";
      std::cout << "\n";
    }
#endif

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
#ifdef USE_ANDY
    // ANDY: restore agents whose waited-for room now has space; else hold at -1
    for (uint kk = 0; kk < N; ++kk) {
      if (waiting_room[kk] >= 0) {
        int rid = waiting_room[kk];
        // (TEST) do NOT grow stored priority during park - restore exactly what it had
        if (rid < (int)rooms.size() &&
            rooms[rid].current_count < rooms[rid].capacity) {
          // room freed up -> restore
          pri[kk] = orig_pri[kk];
          H->priorities[kk] = pri[kk];
          std::cout << "[RESTORE] agent " << kk << " room " << rid
                    << " freed (now " << rooms[rid].current_count << "/" << rooms[rid].capacity
                    << ") at step " << loop_cnt << ", pri restored to " << pri[kk] << "\n";
          waiting_room[kk] = -1;
        } else {
          H->priorities[kk] = -1.0f;  // still waiting
        }
      }
    }
#endif

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
          if (g_exit_boost) H->priorities[kk] = 999.0f + H->priorities[kk];
      }
      // Room full -> block entry
#ifdef USE_ANDY
#ifndef NO_COUNTER
      // ANDY admission: if entering a full room, drop to -1, mark room, store orig priority
      if (waiting_room[kk] < 0) {
        int fr = full_room_to_enter(A[kk]);
        if (fr >= 0) {
          static int chits = 0; chits++;
          if (chits<=5) std::cout << "[COUNTER][ANDY] agent " << kk
                                  << " waits room " << fr << " step " << loop_cnt << "\n";
          orig_pri[kk] = pri[kk];     // remember what it had
          waiting_room[kk] = fr;      // mark which room
          std::cout << "[PARK] agent " << kk << " -> waits room " << fr
                    << " (cap " << rooms[fr].capacity << ", now " << rooms[fr].current_count
                    << ") at step " << loop_cnt << ", orig_pri=" << orig_pri[kk] << "\n";
          pri[kk] = -1.0f;
          H->priorities[kk] = -1.0f;  // lowest, below goals
        }
      }
#endif
#else
      if (approaching_full_room(A[kk])) {
        static int chits = 0; chits++;
        if (chits<=5) std::cout << "[COUNTER] agent " << kk << " step " << loop_cnt << "\n";
        H->priorities[kk] = 0.0001f;
      }
#endif
    }
    if (loop_cnt>=107 && loop_cnt<=130 && rooms.size()>0)
      std::cout << "[ROOM0] step " << loop_cnt << " count=" << rooms[0].current_count
                << "/" << rooms[0].capacity << " agent22_waiting=" << waiting_room[22] << "\n";
    // Re-sort
    std::iota(H->order.begin(), H->order.end(), 0);
    std::sort(H->order.begin(), H->order.end(),
              [&](uint i, uint j) {
                return H->priorities[i] > H->priorities[j];
              });
    // ─────────────────────────────────────────────────────────────

    int p2_attempt = 0; bool p2_redo = true; int p2_prev_pair = -1;
    bool p2_first = true;
    while (p2_redo) {
      p2_redo = false;
      if (!p2_first)   // first pass already sorted above; only re-sort on Part 2 re-attempts
        std::sort(H->order.begin(), H->order.end(),
                  [&](uint i, uint j){ return H->priorities[i] > H->priorities[j]; });
      p2_first = false;
      for (auto k : H->order) {
        auto* a = A[k];
        if (a->v_next != nullptr) continue;
        auto result = funcPIBT(a, a, H);
#ifndef NO_SWAP
        if (result.second != INT_MAX && result.second != -1) {
          std::cout << "[MIKE-SWAP] step " << loop_cnt << ": agent " << k
                    << " (pri " << H->priorities[k] << ") was blocked, swapping priority with blocker agent "
                    << result.second << " (pri " << H->priorities[(uint)result.second]
                    << ") -> blocker promoted to move first\n";
          std::swap(pri[k], pri[(uint)result.second]);
          std::swap(H->priorities[k], H->priorities[(uint)result.second]);
        }
#endif
      }
#if defined(USE_ANDY) && !defined(NO_COUNTER) && !defined(NO_PART2)
      if (p2_attempt < 30) {
        int stuck = -1; float bestp = -1e9;
        for (uint i = 0; i < N; ++i) {
          if (A[i]->v_now != A[i]->v_next) continue;
          if (A[i]->v_now == ins->goals[i]) continue;
          int rid = cell_to_room[A[i]->v_now->id];
          if (rid < 0) continue;
          if (H->priorities[i] > bestp) { bestp = H->priorities[i]; stuck = (int)i; }
        }
        if (stuck >= 0) {
          int rid = cell_to_room[A[stuck]->v_now->id];
          int promote = -1; float bp = -1e9;
          for (uint j = 0; j < N; ++j) {
            if (cell_to_room[A[j]->v_now->id] != rid) continue;
            if (cell_to_room[ins->goals[j]->id] == rid) continue;
            if ((int)j == stuck) continue;
            if (H->priorities[j] > bp) { bp = H->priorities[j]; promote = (int)j; }
          }
          int pair = stuck*100000 + promote;
          if (promote >= 0 && pair != p2_prev_pair) {
            std::swap(pri[stuck], pri[promote]);
            std::swap(H->priorities[stuck], H->priorities[promote]);
            p2_prev_pair = pair; p2_attempt++;
            for (auto* a : A) {
              if (a->v_next != nullptr) { occupied_next[a->v_next->id] = nullptr; a->v_next = nullptr; }
            }
            if (p2_attempt <= 3)
              std::cout << "[PART2] promote agent " << promote << " over " << stuck
                        << " in room " << rid << " step " << loop_cnt
                        << " (attempt " << p2_attempt << ")\n";
            p2_redo = true;
          }
        }
      }
#endif
    }

    Config C_new(N);
    for (auto* a : A) {
      if (a->v_next == nullptr) a->v_next = a->v_now;
      C_new[a->id] = a->v_next;
    }

    if (C_new == solution.back()) {
#if defined(USE_ANDY) && !defined(NO_COUNTER)
      bool parked=false; for(uint i=0;i<N;i++) if(waiting_room[i]>=0){parked=true;break;}

      if(parked && ++frozen_count<g_frozen_cap){ solution.push_back(C_new); goto after_dl_break; }
      frozen_count=0;
#endif
      solver_info(1, "deadlock at step ", loop_cnt);
      break;
    }
    after_dl_break:;

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
#if defined(USE_ANDY) && !defined(NO_COUNTER)
    {
      static std::ofstream vdump; static bool vinit=false;
      if(!vinit){
        vdump.open("/tmp/live_viz.txt", std::ios::out);
        int Wd=ins->G.width;
        vdump<<"goals:";
        for(uint i=0;i<N;i++){int k=ins->goals[i]->index; vdump<<(k%Wd)<<","<<(k/Wd)<<";";}
        vdump<<"\nroom0:";
        if(!rooms.empty()) for(int cid:rooms[0].cells){int ix=(int)ins->G.V[cid]->index; vdump<<(ix%Wd)<<","<<(ix/Wd)<<";";}
        vdump<<"\n"; vinit=true;
      }
      int Wd=ins->G.width;
      vdump<<loop_cnt<<"|pos:";
      for(auto* a:A){int k=a->v_now->index; vdump<<(k%Wd)<<","<<(k/Wd)<<";";}
      vdump<<"|pri:";
      for(uint i=0;i<N;i++) vdump<<pri[i]<<";";
      vdump<<"|fires:"<<g_fire_count<<";agent:"<<g_fire_agent;
      g_fire_count=0; g_fire_agent=-1;
      vdump<<"\n"; vdump.flush();
      // DISABLED viz cap (was artificially failing full cases): if(loop_cnt>300){ solver_info(1,"viz cap"); break; }
    }
#endif

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

#ifndef USE_ANDY
void Planner::detect_rooms(int width, int height)
{
  const int W = (int)ins->G.width, H = (int)ins->G.height;
  cell_to_room.assign(V_size, -1);
  cell_to_rooms.assign(V_size, {});
  rooms.clear();

  auto& V = ins->G.V;
  int n = (int)V_size;

  // ---- iterative Tarjan: articulation points + biconnected components ----
  std::vector<int> disc(n,0), low(n,0); int timer=0;
  std::vector<bool> isArt(n,false);
  std::vector<std::pair<int,int>> estk;
  std::vector<std::vector<int>> bccs;

  for (int s=0; s<n; ++s) {
    if (disc[s]) continue;
    std::vector<std::array<int,3>> st; st.push_back({s,-1,0});
    disc[s]=low[s]=++timer;
    std::vector<int> cnt(n,0);
    while(!st.empty()){
      int u=st.back()[0], parent=st.back()[1]; int& idx=st.back()[2];
      if(idx < (int)V[u]->neighbor.size()){
        int v=(int)V[u]->neighbor[idx]->id; idx++;
        if(v==parent) continue;
        if(!disc[v]){ estk.push_back({u,v}); cnt[u]++; disc[v]=low[v]=++timer; st.push_back({v,u,0}); }
        else if(disc[v]<disc[u]){ estk.push_back({u,v}); low[u]=std::min(low[u],disc[v]); }
      } else {
        st.pop_back();
        if(!st.empty()){
          int p=st.back()[0]; low[p]=std::min(low[p],low[u]); int pp=st.back()[1];
          if((pp!=-1&&low[u]>=disc[p])||(pp==-1&&cnt[p]>1)){
            isArt[p]=true;
            std::set<int> comp;
            while(!estk.empty()){auto e=estk.back();estk.pop_back();
              comp.insert(e.first);comp.insert(e.second);
              if(e.first==p&&e.second==u)break;}
            bccs.push_back(std::vector<int>(comp.begin(),comp.end()));
          }
        }
      }
    }
    if(!estk.empty()){std::set<int>comp;while(!estk.empty()){auto e=estk.back();estk.pop_back();comp.insert(e.first);comp.insert(e.second);}bccs.push_back(std::vector<int>(comp.begin(),comp.end()));}
  }

  auto rc = [&](int cid){ int idx=(int)V[cid]->index; return std::make_pair(idx/W, idx%W); };
  auto has2x2 = [&](std::vector<int>& comp){
    std::set<std::pair<int,int>> s; for(int i:comp) s.insert(rc(i));
    for(int i:comp){ auto p=rc(i); int r=p.first,c=p.second;
      if(s.count({r,c})&&s.count({r+1,c})&&s.count({r,c+1})&&s.count({r+1,c+1})) return true; }
    return false; };
  auto border = [&](std::vector<int>& comp){
    for(int i:comp){ auto p=rc(i); if(p.first==0||p.first==H-1||p.second==0||p.second==W-1) return true; }
    return false; };

  int rid=0;
  for(auto& comp : bccs){
    if(!has2x2(comp)) continue;       // non-1-wide rule
    if(border(comp)) continue;        // outside
    int arts=0; for(int i:comp) if(isArt[i]) arts++;
    RoomInfo room(rid);
    for(int i:comp){
      room.cells.push_back(i);
      cell_to_rooms[i].push_back(rid);
      if(cell_to_room[i]<0) cell_to_room[i]=rid;
      if(isArt[i]) room.entrances.push_back(i);
    }
    room.num_doors = arts;
    room.capacity = (int)room.cells.size();
    room.current_count = 0;
    rooms.push_back(room);
    rid++;
  }

  std::cout << "[ROOM] Detected " << rooms.size() << " rooms\n";
  for(auto& r : rooms)
    std::cout << "[ROOM]   Room " << r.id << ": " << r.cells.size()
              << " cells, doors=" << r.num_doors << ", cap=" << r.capacity << "\n";
}
#endif

#ifdef USE_ANDY
void Planner::detect_rooms(int width, int height)
{
  const int W = (int)ins->G.width;
  cell_to_room.assign(V_size, -1);
  cell_to_rooms.assign(V_size, {});
  rooms.clear();
  auto& V = ins->G.V;

  auto rc = [&](int id){ int idx=(int)V[id]->index; return std::make_pair(idx/W, idx%W); };
  auto deg = [&](int id){ return (int)V[id]->neighbor.size(); };

  // reach test: can a's neighbours reach each other with cell `bl` removed
  auto reach = [&](int a, int b, int bl){
    if(a==bl||b==bl) return false;
    std::set<int> seen={a}; std::vector<int> st={a};
    while(!st.empty()){int x=st.back();st.pop_back(); if(x==b)return true;
      for(auto* nb:V[x]->neighbor){int y=(int)nb->id; if(y!=bl&&!seen.count(y)){seen.insert(y);st.push_back(y);}}}
    return false; };
  auto floodBlk = [&](int s, std::set<int>& bl){
    std::set<int> seen; if(bl.count(s))return seen; seen.insert(s); std::vector<int> st={s};
    while(!st.empty()){int x=st.back();st.pop_back();
      for(auto* nb:V[x]->neighbor){int y=(int)nb->id; if(!bl.count(y)&&!seen.count(y)){seen.insert(y);st.push_back(y);}}}
    return seen; };

  // thin cells = degree <= 2
  std::set<int> thin;
  for(auto* v:V) if(deg((int)v->id)<=2) thin.insert((int)v->id);

  // group thin cells into chains, andy method
  std::vector<std::set<int>> roomsets;
  std::vector<int> roomOutside;  // 1 = giant/outside side (don't gate entry)
  std::set<int> seent;
  for(int s : thin){
    if(seent.count(s)) continue;
    std::set<int> ch={s}; seent.insert(s); std::vector<int> st={s};
    while(!st.empty()){int x=st.back();st.pop_back();
      for(auto* nb:V[x]->neighbor){int y=(int)nb->id; if(thin.count(y)&&!ch.count(y)){ch.insert(y);seent.insert(y);st.push_back(y);}}}
    std::set<int> eps;
    for(int cc:ch) for(auto* nb:V[cc]->neighbor){int y=(int)nb->id; if(!thin.count(y))eps.insert(y);}
    if(eps.size()!=2) continue;
    auto it=eps.begin(); int a=*it++; int b=*it;
    std::set<int> chset(ch.begin(),ch.end());
    auto ra=floodBlk(a,chset);
    if(!ra.count(b)){ auto rb=floodBlk(b,chset);
      // INNER side = the side NOT touching the map outer boundary
      auto touchesOut=[&](std::set<int>& reg){
        for(int cid:reg){ int idx=(int)V[cid]->index; int rr=idx/W, ccc=idx%W;
          if(rr==0||ccc==0||rr==(int)ins->G.height-1||ccc==W-1) return true; }
        return false; };
      bool aOut=touchesOut(ra), bOut=touchesOut(rb);
      // DOUBLE-SIDED: keep BOTH sides. Tag the side the old picker would DROP as outside.
      int realIsA;
      if(aOut&&!bOut) realIsA=0;        // ra touches outside -> rb real
      else if(bOut&&!aOut) realIsA=1;   // rb touches outside -> ra real
      else realIsA=(ra.size()<=rb.size())?1:0;  // neither/both -> smaller real
      if(realIsA){ roomsets.push_back(ra); roomOutside.push_back(0);
                   roomsets.push_back(rb); roomOutside.push_back(1); }
      else { roomsets.push_back(rb); roomOutside.push_back(0);
             roomsets.push_back(ra); roomOutside.push_back(1); } }
  }

  int R=(int)roomsets.size();
  std::vector<std::set<int>> exits(R);
  for(int i=0;i<R;i++)
    for(int cid:roomsets[i])
      for(auto* nb:V[cid]->neighbor){int y=(int)nb->id;
        if(!roomsets[i].count(y)) exits[i].insert(y);}
  std::vector<int> parent(R,-1);
  std::vector<bool> combined(R,false);
  for(int i=0;i<R;i++){
    int best=-1;
    for(int j=0;j<R;j++){
      if(i==j||roomsets[j].size()<=roomsets[i].size())continue;
      bool contains=true; for(int p:roomsets[i]) if(!roomsets[j].count(p)){contains=false;break;}
      if(!contains)continue;
      if(best<0||roomsets[j].size()<roomsets[best].size()) best=j;  // pure containment
    }
    parent[i]=best;
  }
  // room j is COMBINED if it contains 2+ rooms that are DISJOINT from each other
  // (it spans separate peer rooms, e.g. rooms+hallway merged) -> not a true parent.
  for(int j=0;j<R;j++){
    std::vector<int> contained;
    for(int i=0;i<R;i++){
      if(i==j||roomsets[j].size()<=roomsets[i].size())continue;
      bool contains=true; for(int p:roomsets[i]) if(!roomsets[j].count(p)){contains=false;break;}
      if(contains) contained.push_back(i);
    }
    // check for a disjoint pair among contained rooms
    for(size_t a=0;a<contained.size()&&!combined[j];a++)
      for(size_t b=a+1;b<contained.size()&&!combined[j];b++){
        bool overlap=false;
        for(int p:roomsets[contained[a]]) if(roomsets[contained[b]].count(p)){overlap=true;break;}
        if(!overlap) combined[j]=true;   // two disjoint rooms inside j -> combined
      }
  }
  // a combined room is NOT a true parent: clear it from parent[]
  std::vector<int> depth(R,1);
  for(int i=0;i<R;i++){int d=1,p=parent[i]; while(p>=0){d++;p=parent[p];} depth[i]=d;}

  for(int i=0;i<R;i++){
    RoomInfo room(i);
    for(int cid:roomsets[i]){
      room.cells.push_back(cid);
      cell_to_rooms[cid].push_back(i);
      if(cell_to_room[cid]<0) cell_to_room[cid]=i;  // primary; refined below
    }
    room.capacity = (int)roomsets[i].size();
    room.current_count = 0;
    room.depth = depth[i];
    room.is_combined = combined[i];
    room.parent = parent[i];
    room.outside = roomOutside[i];
    rooms.push_back(room);
  }
  // primary room per cell = DEEPEST (highest depth) room it belongs to
  for(int cid=0; cid<(int)V_size; ++cid){
    int best=-1, bd=-1;
    for(int rid : cell_to_rooms[cid]) if(rooms[rid].depth>bd){bd=rooms[rid].depth;best=rid;}
    cell_to_room[cid]=best;
  }

  std::cout << "[ROOM][ANDY] Detected " << rooms.size() << " rooms\n";
  for(auto& r : rooms)
    std::cout << "[ROOM]   Room " << r.id << ": " << r.cells.size()
              << " cells, depth=" << r.depth << ", parent=" << r.parent
              << ", cap=" << r.capacity << (r.is_combined?" [COMBINED]":"") << (r.outside?" [OUTSIDE]":"") << "\n";
  // dump rooms to file for the visualizer (x,y per cell)
  {
    std::ofstream rf("rooms.txt", std::ios::out);
    int Wd = ins->G.width;
    for(auto& r : rooms){
      rf << "room " << r.id << " cap " << r.capacity
         << " outside " << r.outside << " combined " << (r.is_combined?1:0)
         << " depth " << r.depth << " cells";
      for(int cid : r.cells){ int idx=(int)V[cid]->index; rf << " " << (idx%Wd) << "," << (idx/Wd); }
      rf << "\n";
    }
  }
}
#endif


void Planner::update_room_counts()
{
  for (auto& r : rooms) r.current_count = 0;
  for (auto* a : A) {
    if (a->v_now == nullptr) continue;
#ifdef USE_ANDY
    // CASCADE: count agent into EVERY room its cell belongs to
    for (int rid : cell_to_rooms[a->v_now->id])
      if (rid >= 0 && rid < (int)rooms.size())
        rooms[rid].current_count++;
#else
    int rid = cell_to_room[a->v_now->id];
    if (rid >= 0 && rid < (int)rooms.size())
      rooms[rid].current_count++;
#endif
  }
}

bool Planner::approaching_full_room(Agent* ai)
{
  return full_room_to_enter(ai) >= 0;
}
// Returns the id of a full room the agent would ENTER (it is currently outside),
// or -1 if none. Under USE_ANDY this checks ALL rooms each neighbour belongs to.
int Planner::full_room_to_enter(Agent* ai)
{
  if (rooms.empty() || ai->v_now == nullptr) return -1;
#ifdef USE_ANDY
  // agent must not already be inside the room it's about to enter
  for (auto* nb : ai->v_now->neighbor) {
    for (int rid : cell_to_rooms[nb->id]) {
      if (rid < 0) continue;
      // skip rooms the agent is already in (not "entering")
      bool already = false;
      for (int mine : cell_to_rooms[ai->v_now->id]) if (mine == rid) already = true;
      if (already) continue;
      if (rooms[rid].is_combined) continue;   // combined = observation counter, not a gate
      if (rooms[rid].outside) continue;       // giant/outside side = never gates entry
      if (rooms[rid].current_count >= rooms[rid].capacity) {
        g_fire_count++; g_fire_agent = (int)ai->id;
        std::cout << "[ADMIT] agent " << (int)ai->id << " at " << ai->v_now->id
                  << " BLOCKED entering room " << rid << " ("
                  << rooms[rid].current_count << "/" << rooms[rid].capacity << ")\n";
        return rid; }
    }
  }
  return -1;
#else
  int current_room = cell_to_room[ai->v_now->id];
  if (current_room >= 0) return -1;
  for (auto* nb : ai->v_now->neighbor) {
    int rid = cell_to_room[nb->id];
    if (rid < 0) continue;
    if (rooms[rid].current_count >= rooms[rid].capacity) return rid;
  }
  return -1;
#endif
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
