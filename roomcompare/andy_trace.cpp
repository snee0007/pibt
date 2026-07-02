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
  auto emit=[&](string msg,string det,string col,set<pair<int,int>>cells){
    if(!first)printf(",\n");first=false;
    printf("{\"phase\":\"classify\",\"msg\":\"%s\",\"detail\":\"%s\",\"color\":\"%s\",\"cells\":[",msg.c_str(),det.c_str(),col.c_str());
    bool f=true;for(auto&p:cells){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=false;}printf("]}");
  };

  set<pair<int,int>> seent; int ci=0;
  for(auto&s:thin){
    if(seent.count(s))continue;
    set<pair<int,int>>ch={s}; seent.insert(s); vector<pair<int,int>>st={s};
    while(st.size()){auto x=st.back();st.pop_back();
      for(auto&y:m.nbr[x])if(thin.count(y)&&!ch.count(y)){ch.insert(y);seent.insert(y);st.push_back(y);}}
    set<pair<int,int>>eps;
    for(auto&cc:ch)for(auto&y:m.nbr[cc])if(!thin.count(y))eps.insert(y);

    // STEP 1: show the chain itself
    char m1[100];snprintf(m1,100,"chain %d: corridor of %d cells, %d endpoints",ci,(int)ch.size(),(int)eps.size());
    emit(m1,"This is a corridor chain (all thin cells grouped). Yellow = the chain. Next: check its endpoints.","chain",ch);

    if(eps.size()!=2){
      char m2[100];snprintf(m2,100,"chain %d: SKIP (%d endpoints, need exactly 2)",ci,(int)eps.size());
      emit(m2,"Andy's rule: only chains with exactly 2 endpoints are tested. This one is skipped (dead-end stub or junction).","corridor",ch);
      ci++;continue;
    }
    auto it=eps.begin();auto a=*it++;auto b=*it;

    // STEP 2: show endpoints A and B
    set<pair<int,int>> abset={a,b};
    char m3[100];snprintf(m3,100,"chain %d: endpoints A=(%d,%d) B=(%d,%d)",ci,a.first,a.second,b.first,b.second);
    emit(m3,"The two endpoints of the chain. Now flood from A with the chain blocked. Can it reach B?","flood",abset);

    auto ra=flood(a,ch);
    if(ra.count(b)){
      // STEP 3a: A reaches B = not a room
      char m4[100];snprintf(m4,100,"chain %d: A reached B -> NOT a room",ci);
      emit(m4,"Flood from A (blue) reached B. There is a way around -> not sealed -> not a room.","flood",ra);
    } else {
      auto rb=flood(b,ch);auto rs=(ra.size()<=rb.size())?ra:rb;
      // STEP 3b: sealed = room
      char m5[100];snprintf(m5,100,"chain %d: A could NOT reach B -> ROOM %d cells",ci,(int)rs.size());
      emit(m5,"Flood from A blocked by the chain -> sealed region -> this is a ROOM (green). Counter = these cells.","room",rs);
    }
    ci++;
  }
  printf("\n]}\n");
}
