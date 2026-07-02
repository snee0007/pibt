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

  // JSON header
  printf("{\n\"H\":%d,\"W\":%d,\n\"grid\":[",m.H,m.W);
  for(int r=0;r<m.H;r++){printf("\"");for(int cc=0;cc<m.W;cc++)printf("%c",(cc<(int)m.g[r].size()&&m.g[r][cc]=='.')?'.':'@');printf("\"%s",r==m.H-1?"":",");}
  printf("],\n\"steps\":[\n");
  bool first=true;
  auto emit=[&](string msg,string det,string col,set<pair<int,int>>cells,pair<int,int>door){
    if(!first)printf(",\n");first=false;
    printf("{\"phase\":\"classify\",\"msg\":\"%s\",\"detail\":\"%s\",\"color\":\"%s\",\"cells\":[",msg.c_str(),det.c_str(),col.c_str());
    bool f=true;for(auto&p:cells){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=false;}
    printf("],\"door\":[%d,%d]}",door.first,door.second);
  };

  set<pair<int,int>> seent; int rooms=0,skip=0,ci=0;
  for(auto&s:thin){
    if(seent.count(s))continue;
    set<pair<int,int>>ch={s}; seent.insert(s); vector<pair<int,int>>st={s};
    while(st.size()){auto x=st.back();st.pop_back();
      for(auto&y:m.nbr[x])if(thin.count(y)&&!ch.count(y)){ch.insert(y);seent.insert(y);st.push_back(y);}}
    set<pair<int,int>>eps;
    for(auto&cc:ch)for(auto&y:m.nbr[cc])if(!thin.count(y))eps.insert(y);
    pair<int,int> firstcell=*ch.begin();
    if(eps.size()!=2){
      skip++;
      char msg[80];snprintf(msg,80,"chain %d: %d endpoints -> SKIP",ci,(int)eps.size());
      emit(msg,"Chain does not have exactly 2 endpoints. Skipped.","corridor",ch,firstcell);
      ci++;continue;
    }
    auto it=eps.begin();auto a=*it++;auto b=*it;
    auto ra=flood(a,ch);
    if(!ra.count(b)){
      auto rb=flood(b,ch);auto&rs=(ra.size()<=rb.size())?ra:rb;
      bool phantom = rs.size()>20;   // big trapped chunk = likely phantom
      char msg[100];snprintf(msg,100,"chain %d: %s %d cells",ci,phantom?"PHANTOM":"room",(int)rs.size());
      char det[200];snprintf(det,200,"A=(%d,%d) B=(%d,%d). Flood from A blocked -> sealed %d cells. %s",
        a.first,a.second,b.first,b.second,(int)rs.size(),
        phantom?"PHANTOM: trapped by other removed hallway pieces.":"room.");
      emit(msg,det,phantom?"phantom":"room",rs,firstcell);
      rooms++;
    } else {
      char msg[80];snprintf(msg,80,"chain %d: A reaches B -> no room",ci);
      emit(msg,"Flood from A reached B (way around) -> not a room.","corridor",ch,firstcell);
    }
    ci++;
  }
  printf("\n]}\n");
}
