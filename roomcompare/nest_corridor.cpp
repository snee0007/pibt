#include "common.h"
#include <functional>
Map m;
int main(int c,char**v){
 m.load(v[1]);
 set<pair<int,int>> thin;
 for(auto&rc:m.cells) if(m.deg(rc)<=2) thin.insert(rc);
 auto flood=[&](pair<int,int>s,set<pair<int,int>>&bl){
   set<pair<int,int>>seen; if(bl.count(s))return seen;
   seen.insert(s); vector<pair<int,int>>st={s};
   while(st.size()){auto x=st.back();st.pop_back();
     for(auto&y:m.nbr[x])if(!bl.count(y)&&!seen.count(y)){seen.insert(y);st.push_back(y);}}
   return seen;};
 auto touchesOut=[&](set<pair<int,int>>&reg){
   for(auto&p:reg)for(auto&y:m.nbr[p])
     if(y.first==0||y.second==0||y.first==m.H-1||y.second==m.W-1)return true;
   return false;};
 vector<set<pair<int,int>>> rooms;
 std::vector<int> outsideTag;
 vector<set<pair<int,int>>> roomCorridor;
 vector<pair<int,int>> roomGate;
 set<pair<int,int>> seent;
 for(auto&s:thin){
   if(seent.count(s))continue;
   set<pair<int,int>>ch={s}; seent.insert(s); vector<pair<int,int>>st={s};
   while(st.size()){auto x=st.back();st.pop_back();
     for(auto&y:m.nbr[x])if(thin.count(y)&&!ch.count(y)){ch.insert(y);seent.insert(y);st.push_back(y);}}
   set<pair<int,int>>eps;
   for(auto&cc:ch)for(auto&y:m.nbr[cc])if(!thin.count(y))eps.insert(y);
   if(eps.size()==2){
     auto it=eps.begin();auto a=*it++;auto b=*it;
     auto ra=flood(a,ch);
     if(!ra.count(b)){auto rb=flood(b,ch);
       bool aO=touchesOut(ra), bO=touchesOut(rb);
       int realIsA;
       if(aO&&!bO) realIsA=0;
       else if(bO&&!aO) realIsA=1;
       else realIsA=(ra.size()<=rb.size())?1:0;
       auto gateFor=[&](pair<int,int> endpoint)->pair<int,int>{
         for(auto&y:m.nbr[endpoint]) if(ch.count(y)) return y;
         return endpoint;
       };
       pair<int,int> gateA=gateFor(a);
       pair<int,int> gateB=gateFor(b);
       if(realIsA){
         rooms.push_back(ra); outsideTag.push_back(0); roomCorridor.push_back(ch); roomGate.push_back(gateA);
         rooms.push_back(rb); outsideTag.push_back(1); roomCorridor.push_back(ch); roomGate.push_back(gateB);
       } else {
         rooms.push_back(rb); outsideTag.push_back(0); roomCorridor.push_back(ch); roomGate.push_back(gateB);
         rooms.push_back(ra); outsideTag.push_back(1); roomCorridor.push_back(ch); roomGate.push_back(gateA);
       }
     }
   }
 }
 int R=rooms.size();
 vector<int> parent(R,-1);
 for(int i=0;i<R;i++){int best=-1;
   for(int j=0;j<R;j++){ if(i==j||rooms[j].size()<=rooms[i].size())continue;
     bool ct=true; for(auto&p:rooms[i]) if(!rooms[j].count(p)){ct=false;break;}
     if(ct){ if(best<0||rooms[j].size()<rooms[best].size()) best=j; } }
   parent[i]=best; }
 vector<int> depth(R,1);
 for(int i=0;i<R;i++){int d=1,p=parent[i]; while(p>=0){d++;p=parent[p];} depth[i]=d;}
 std::vector<int> outside=outsideTag;
 printf("{\n\"H\":%d,\"W\":%d,\n\"grid\":[",m.H,m.W);
 for(int r=0;r<m.H;r++){printf("\"");for(int cc=0;cc<m.W;cc++)printf("%c",(cc<(int)m.g[r].size()&&m.g[r][cc]=='.')?'.':'@');printf("\"%s",r==m.H-1?"":",");}
 printf("],\n\"steps\":[\n");
 bool first=true;
 for(int i=0;i<R;i++){
   if(!first)printf(",\n");first=false;
   string col = (depth[i]==1?"room":(depth[i]==2?"room2":"phantom"));
   printf("{\"msg\":\"room %d: %d cells, corridor %d cells, gate [%d,%d]\",",
     i,(int)rooms[i].size(),(int)roomCorridor[i].size(),roomGate[i].first,roomGate[i].second);
   printf("\"detail\":\"room_cap=%d, room+corridor=%d, gate at [%d,%d]\",",
     (int)rooms[i].size(),(int)(rooms[i].size()+roomCorridor[i].size()),roomGate[i].first,roomGate[i].second);
   printf("\"outside\":%d,\"color\":\"%s\",\"roomid\":%d,",outside[i],col.c_str(),i);
   printf("\"cells\":[");
   bool f=true;for(auto&p:rooms[i]){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=false;}printf("],");
   printf("\"corridor\":[");
   f=true;for(auto&p:roomCorridor[i]){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=false;}printf("],");
   printf("\"gate\":[%d,%d]}",roomGate[i].first,roomGate[i].second);
 }
 printf("\n]}\n");
}
