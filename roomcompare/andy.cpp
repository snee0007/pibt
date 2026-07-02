#include "common.h"
int main(int c,char**v){
  Map m; m.load(v[1]);
  set<pair<int,int>> thin;
  for(auto&rc:m.cells) if(m.deg(rc)<=2) thin.insert(rc);
  auto flood=[&](pair<int,int>s,set<pair<int,int>>&bl){
    set<pair<int,int>>seen; if(bl.count(s))return seen;
    seen.insert(s); vector<pair<int,int>>st={s};
    while(st.size()){auto x=st.back();st.pop_back();
      for(auto&y:m.nbr[x])if(!bl.count(y)&&!seen.count(y)){seen.insert(y);st.push_back(y);}}
    return seen;};
  set<pair<int,int>> seent; int rooms=0,skip=0;
  for(auto&s:thin){
    if(seent.count(s))continue;
    set<pair<int,int>>ch={s}; seent.insert(s); vector<pair<int,int>>st={s};
    while(st.size()){auto x=st.back();st.pop_back();
      for(auto&y:m.nbr[x])if(thin.count(y)&&!ch.count(y)){ch.insert(y);seent.insert(y);st.push_back(y);}}
    set<pair<int,int>>eps;
    for(auto&cc:ch)for(auto&y:m.nbr[cc])if(!thin.count(y))eps.insert(y);
    if(eps.size()!=2){skip++;continue;}
    auto it=eps.begin();auto a=*it++;auto b=*it;
    auto ra=flood(a,ch);
    if(!ra.count(b)){auto rb=flood(b,ch);auto&rs=(ra.size()<=rb.size())?ra:rb;
      printf("  ANDY room: %d cells\n",(int)rs.size());rooms++;}
  }
  printf("ANDY  (%s): %d rooms, %d chains skipped(!=2 ends)\n",v[1],rooms,skip);
}
