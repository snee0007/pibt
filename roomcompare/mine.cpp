#include "common.h"
int main(int c,char**v){
  Map m; m.load(v[1]);
  auto reach=[&](pair<int,int>a,pair<int,int>b,pair<int,int>bl){
    if(a==bl||b==bl)return false;
    set<pair<int,int>>seen={a};vector<pair<int,int>>st={a};
    while(st.size()){auto x=st.back();st.pop_back();if(x==b)return true;
      for(auto&y:m.nbr[x])if(!(y==bl)&&!seen.count(y)){seen.insert(y);st.push_back(y);}}
    return false;};
  set<pair<int,int>>doors;
  for(auto&rc:m.cells){if(m.deg(rc)>2)continue;auto&ns=m.nbr[rc];
    for(size_t i=0;i<ns.size();i++)for(size_t j=i+1;j<ns.size();j++)
      if(!reach(ns[i],ns[j],rc))doors.insert(rc);}
  map<pair<int,int>,int>comp;int cid=0;
  for(auto&s:m.cells){if(doors.count(s)||comp.count(s))continue;
    comp[s]=cid;vector<pair<int,int>>st={s};
    while(st.size()){auto x=st.back();st.pop_back();
      for(auto&y:m.nbr[x])if(!doors.count(y)&&!comp.count(y)){comp[y]=cid;st.push_back(y);}}cid++;}
  vector<vector<pair<int,int>>>cc(cid);vector<set<pair<int,int>>>cd(cid);
  for(auto&rc:m.cells)if(comp.count(rc)){cc[comp[rc]].push_back(rc);
    for(auto&y:m.nbr[rc])if(doors.count(y))cd[comp[rc]].insert(y);}
  int big=0;for(int k=0;k<cid;k++)if(cc[k].size()>cc[big].size())big=k;
  int rooms=0;
  for(int k=0;k<cid;k++){if(k==big)continue;bool hi=false;
    for(auto&rc:cc[k])if(m.deg(rc)>=3)hi=true;
    if(cd[k].size()==1&&hi){printf("  MINE room: %d cells topleft(%d,%d)\n",(int)cc[k].size(),cc[k][0].first,cc[k][0].second);rooms++;}}
  printf("MINE  (%s): %d rooms\n",v[1],rooms);
}
