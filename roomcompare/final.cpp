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
  auto has2x2=[&](vector<pair<int,int>>&cells){
    set<pair<int,int>>s(cells.begin(),cells.end());
    for(auto&p:cells){int r=p.first,col=p.second;
      if(s.count({r,col})&&s.count({r+1,col})&&s.count({r,col+1})&&s.count({r+1,col+1}))return true;}
    return false;};
  auto border=[&](vector<pair<int,int>>&cells){
    for(auto&p:cells) if(p.first==0||p.first==m.H-1||p.second==0||p.second==m.W-1) return true;
    return false;};
  int single=0,multi=0,corr=0,hub=0,out=0;
  printf("=== %s ===\n",v[1]);
  for(int k=0;k<cid;k++){
    int d=cd[k].size(); bool wide=has2x2(cc[k]); int sz=cc[k].size();
    string type;
    if(border(cc[k])){ type="OUTSIDE"; out++; }
    else if(!wide){ type="CORRIDOR(cap0)"; corr++; }
    else if(d==1){ type="SINGLE-DOOR ROOM"; single++; }
    else if(d==2){ type="TWO-DOOR ROOM"; multi++; }
    else { type="HUB"; hub++; }
    if(type[0]!='O') printf("  [%s] %d cells, %d door(s)\n",type.c_str(),sz,d);
  }
  printf("SUMMARY: %d single-door | %d two-door | %d hubs | %d corridors | %d outside\n\n",single,multi,hub,corr,out);
}
