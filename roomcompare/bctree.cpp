#include "common.h"
Map m;
int idC=0; map<pair<int,int>,int> id; vector<pair<int,int>> cells;
vector<vector<int>> adj;
vector<int> disc, low; int timer=0;
vector<bool> isArt;
vector<pair<int,int>> estk;
vector<vector<int>> bccs;
void tarjanIter(int s){
  vector<array<int,3>> st; st.push_back({s,-1,0});
  disc[s]=low[s]=++timer; vector<int> cnt(idC,0);
  while(!st.empty()){
    int u=st.back()[0], parent=st.back()[1], &idx=st.back()[2];
    if(idx<(int)adj[u].size()){
      int v=adj[u][idx]; idx++;
      if(v==parent) continue;
      if(!disc[v]){ estk.push_back({u,v}); cnt[u]++; disc[v]=low[v]=++timer; st.push_back({v,u,0}); }
      else if(disc[v]<disc[u]){ estk.push_back({u,v}); low[u]=min(low[u],disc[v]); }
    } else {
      st.pop_back();
      if(!st.empty()){
        int p=st.back()[0]; low[p]=min(low[p],low[u]); int pp=st.back()[1];
        if((pp!=-1&&low[u]>=disc[p])||(pp==-1&&cnt[p]>1)){ isArt[p]=true;
          set<int> comp; while(!estk.empty()){auto e=estk.back();estk.pop_back();comp.insert(e.first);comp.insert(e.second);if(e.first==p&&e.second==u)break;}
          bccs.push_back(vector<int>(comp.begin(),comp.end())); }
      }
    }
  }
  if(!estk.empty()){set<int>comp;while(!estk.empty()){auto e=estk.back();estk.pop_back();comp.insert(e.first);comp.insert(e.second);}bccs.push_back(vector<int>(comp.begin(),comp.end()));}
}
int main(int c,char**v){
  m.load(v[1]);
  for(auto&rc:m.cells){id[rc]=idC++;cells.push_back(rc);}
  adj.assign(idC,{});
  for(auto&rc:m.cells)for(auto&y:m.nbr[rc])adj[id[rc]].push_back(id[y]);
  disc.assign(idC,0);low.assign(idC,0);isArt.assign(idC,false);
  for(int i=0;i<idC;i++)if(!disc[i])tarjanIter(i);
  auto has2x2=[&](vector<int>&comp){set<pair<int,int>>s;for(int i:comp)s.insert(cells[i]);
    for(int i:comp){int r=cells[i].first,cc=cells[i].second;
      if(s.count({r,cc})&&s.count({r+1,cc})&&s.count({r,cc+1})&&s.count({r+1,cc+1}))return true;}return false;};
  auto border=[&](vector<int>&comp){for(int i:comp){auto p=cells[i];
    if(p.first==0||p.first==m.H-1||p.second==0||p.second==m.W-1)return true;}return false;};
  printf("=== %s ===\n",v[1]); int rooms=0;
  for(auto&comp:bccs){int arts=0;for(int i:comp)if(isArt[i])arts++;
    bool wide=has2x2(comp);bool bord=border(comp); if(!wide)continue;
    const char* t=bord?"outside":(arts<=1?"SINGLE-ENTRY ROOM":"nested-parent/multi-entry");
    printf("  BCC: %d cells, %d doors [%s]\n",(int)comp.size(),arts,t);
    if(arts<=1&&!bord)rooms++;}
  printf("single-entry rooms: %d\n\n",rooms);
}
