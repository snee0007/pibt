// Detector that EMITS a step-by-step trace as JSON, for the viz player to replay.
#include "common.h"
int main(int c,char**v){
  Map m; m.load(v[1]);
  // emit map + steps as JSON
  printf("{\n");
  printf("\"H\":%d,\"W\":%d,\n",m.H,m.W);
  // grid
  printf("\"grid\":[");
  for(int r=0;r<m.H;r++){ printf("\"");
    for(int cc=0;cc<m.W;cc++) printf("%c", (cc<(int)m.g[r].size()&&m.g[r][cc]=='.')?'.':'@');
    printf("\"%s", r==m.H-1?"":","); }
  printf("],\n");
  printf("\"steps\":[\n");
  bool first=true;
  auto emit=[&](const string&phase,const string&msg,vector<pair<int,int>>&hi,const string&color){
    if(!first)printf(",\n"); first=false;
    printf("{\"phase\":\"%s\",\"msg\":\"%s\",\"color\":\"%s\",\"cells\":[",phase.c_str(),msg.c_str(),color.c_str());
    for(size_t i=0;i<hi.size();i++)printf("%s[%d,%d]",i?",":"",hi[i].first,hi[i].second);
    printf("]}");
  };
  // --- doors ---
  auto reach=[&](pair<int,int>a,pair<int,int>b,pair<int,int>bl){
    if(a==bl||b==bl)return false;
    set<pair<int,int>>seen={a};vector<pair<int,int>>st={a};
    while(st.size()){auto x=st.back();st.pop_back();if(x==b)return true;
      for(auto&y:m.nbr[x])if(!(y==bl)&&!seen.count(y)){seen.insert(y);st.push_back(y);}}return false;};
  set<pair<int,int>>doors;
  for(auto&rc:m.cells){if(m.deg(rc)>2)continue;auto&ns=m.nbr[rc];bool dr=false;
    for(size_t i=0;i<ns.size();i++)for(size_t j=i+1;j<ns.size();j++)if(!reach(ns[i],ns[j],rc))dr=true;
    if(dr)doors.insert(rc);}
  {vector<pair<int,int>>d(doors.begin(),doors.end());
   emit("doors","Phase 1: door cells found (cut test)",d,"door");}
  // --- components ---
  map<pair<int,int>,int>comp;int cid=0;
  for(auto&s:m.cells){if(doors.count(s)||comp.count(s))continue;comp[s]=cid;vector<pair<int,int>>st={s};
    while(st.size()){auto x=st.back();st.pop_back();
      for(auto&y:m.nbr[x])if(!doors.count(y)&&!comp.count(y)){comp[y]=cid;st.push_back(y);}}cid++;}
  vector<vector<pair<int,int>>>cc(cid);vector<set<pair<int,int>>>cd(cid);
  for(auto&rc:m.cells)if(comp.count(rc)){cc[comp[rc]].push_back(rc);
    for(auto&y:m.nbr[rc])if(doors.count(y))cd[comp[rc]].insert(y);}
  auto has2x2=[&](vector<pair<int,int>>&cells){set<pair<int,int>>s(cells.begin(),cells.end());
    for(auto&p:cells){int r=p.first,col=p.second;
      if(s.count({r,col})&&s.count({r+1,col})&&s.count({r,col+1})&&s.count({r+1,col+1}))return true;}return false;};
  auto border=[&](vector<pair<int,int>>&cells){for(auto&p:cells)
    if(p.first==0||p.first==m.H-1||p.second==0||p.second==m.W-1)return true;return false;};
  for(int k=0;k<cid;k++){int d=cd[k].size();bool wide=has2x2(cc[k]);int sz=cc[k].size();
    string type,col;
    if(border(cc[k])){type="OUTSIDE";col="outside";}
    else if(!wide){type="CORRIDOR cap0";col="corridor";}
    else if(d==1){type="SINGLE-DOOR ROOM";col="room";}
    else if(d==2){type="TWO-DOOR ROOM";col="room2";}
    else{type="HUB "+to_string(d)+"doors";col="hub";}
    char buf[128];snprintf(buf,128,"%s (%d cells, %d doors)",type.c_str(),sz,d);
    emit("classify",buf,cc[k],col);}
  printf("\n]}\n");
}
