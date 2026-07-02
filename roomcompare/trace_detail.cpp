// Detailed trace: shows the per-cell Dijkstra cut test (how each door is chosen),
// then region classification. Same doors as block-cut tree (articulation pts).
#include "common.h"
Map m;
int main(int c,char**v){
 m.load(v[1]);
 auto reach=[&](pair<int,int>a,pair<int,int>b,pair<int,int>bl){
  if(a==bl||b==bl)return false;set<pair<int,int>>seen={a};vector<pair<int,int>>st={a};
  while(st.size()){auto x=st.back();st.pop_back();if(x==b)return true;
   for(auto&y:m.nbr[x])if(!(y==bl)&&!seen.count(y)){seen.insert(y);st.push_back(y);}}return false;};
 // JSON header
 printf("{\n\"H\":%d,\"W\":%d,\n\"grid\":[",m.H,m.W);
 for(int r=0;r<m.H;r++){printf("\"");for(int cc=0;cc<m.W;cc++)printf("%c",(cc<(int)m.g[r].size()&&m.g[r][cc]=='.')?'.':'@');printf("\"%s",r==m.H-1?"":",");}
 printf("],\n\"steps\":[\n");
 bool first=true;
 auto emit=[&](const string&phase,const string&msg,const string&detail,
               vector<pair<int,int>>cells,const string&color,
               pair<int,int>tcell,vector<pair<int,int>>nbrs,bool isdoor){
   if(!first)printf(",\n");first=false;
   printf("{\"phase\":\"%s\",\"msg\":\"%s\",\"detail\":\"%s\",\"color\":\"%s\",",
          phase.c_str(),msg.c_str(),detail.c_str(),color.c_str());
   printf("\"test\":[%d,%d],\"isdoor\":%s,\"nbrs\":[",tcell.first,tcell.second,isdoor?"true":"false");
   for(size_t i=0;i<nbrs.size();i++)printf("%s[%d,%d]",i?",":"",nbrs[i].first,nbrs[i].second);
   printf("],\"cells\":[");
   for(size_t i=0;i<cells.size();i++)printf("%s[%d,%d]",i?",":"",cells[i].first,cells[i].second);
   printf("]}");
 };
 // ---- PHASE 1: per-cell cut test (door selection) ----
 set<pair<int,int>>doors;
 for(auto&rc:m.cells){
   if(m.deg(rc)>2) continue;          // only thin cells can be doors
   auto&ns=m.nbr[rc]; bool isd=false;
   for(size_t i=0;i<ns.size();i++)for(size_t j=i+1;j<ns.size();j++)
     if(!reach(ns[i],ns[j],rc)) isd=true;
   char msg[120],det[200];
   snprintf(msg,120,"Cut test at (%d,%d): %s",rc.first,rc.second,isd?"DOOR":"not a door");
   if(ns.size()>=2)
     snprintf(det,200,"Remove (%d,%d). Can its neighbours reach each other? %s -> %s",
       rc.first,rc.second, isd?"NO":"YES", isd?"this cell is a DOOR (cut)":"not a door, skip");
   else
     snprintf(det,200,"(%d,%d) has <2 open neighbours (dead-end tip)",rc.first,rc.second);
   if(isd)doors.insert(rc);
   vector<pair<int,int>> dv(doors.begin(),doors.end());
   emit("test",msg,det,dv,"door",rc,ns,isd);
 }
 // ---- PHASE 2: regions (remove doors, flood) ----
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
 int rid=0;
 for(int k=0;k<cid;k++){
   int d=cd[k].size(); bool wide=has2x2(cc[k]); bool bord=border(cc[k]); int sz=cc[k].size();
   string col,msg,det;
   if(bord){col="outside";msg="OUTSIDE (touches border)";det="Region touches map edge -> open outside, not a room.";}
   else if(!wide){col="corridor";msg="CORRIDOR (1-wide, cap 0)";det="No 2x2 block -> 1-wide -> corridor, not a room.";}
   else{ col=(d<=1)?"room":"room2";
     char b[100],e[200];snprintf(b,100,"ROOM #%d: %d cells, %d door(s)",rid++,sz,d);
     snprintf(e,200,"Non-1-wide region separated by cut -> ROOM. capacity=%d, doors=%d",sz,d);
     msg=b;det=e;}
   emit("classify",msg,det,cc[k],col,{-1,-1},{},false);
 }
 printf("\n]}\n");
}
