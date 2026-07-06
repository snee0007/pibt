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
 vector<pair<int,int>> roomCtr;   // ROOM counter (near door)
 vector<pair<int,int>> rcCtr;     // RC counter (far/corridor mouth)
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
       auto chainNext=[&](pair<int,int> pt)->pair<int,int>{
         for(auto&y:m.nbr[pt])if(ch.count(y))return y; return pt; };
       // For the REAL room:
       //   room counter = chain cell next to REAL room's endpoint (near door)
       //   rc counter   = the OTHER (outside) endpoint (far / corridor mouth)
       // The outside room gets the mirrored pair (mostly dead, shown for completeness).
       if(realIsA){
         // ra real (endpoint a), rb outside (endpoint b)
         rooms.push_back(ra); outsideTag.push_back(0); roomCorridor.push_back(ch);
           roomCtr.push_back(chainNext(a)); rcCtr.push_back(b);
         rooms.push_back(rb); outsideTag.push_back(1); roomCorridor.push_back(ch);
           roomCtr.push_back(chainNext(b)); rcCtr.push_back(a);
       } else {
         rooms.push_back(rb); outsideTag.push_back(0); roomCorridor.push_back(ch);
           roomCtr.push_back(chainNext(b)); rcCtr.push_back(a);
         rooms.push_back(ra); outsideTag.push_back(1); roomCorridor.push_back(ch);
           roomCtr.push_back(chainNext(a)); rcCtr.push_back(b);
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
 vector<bool> combined(R,false);
 for(int j=0;j<R;j++){ vector<int> cont;
   for(int i=0;i<R;i++){ if(i==j||rooms[j].size()<=rooms[i].size())continue;
     bool ct=true; for(auto&p:rooms[i]) if(!rooms[j].count(p)){ct=false;break;} if(ct)cont.push_back(i);}
   for(size_t a=0;a<cont.size()&&!combined[j];a++)for(size_t b=a+1;b<cont.size()&&!combined[j];b++){
     bool ov=false; for(auto&p:rooms[cont[a]]) if(rooms[cont[b]].count(p)){ov=true;break;}
     if(!ov)combined[j]=true; } }
 vector<int> depth(R,1);
 for(int i=0;i<R;i++){int d=1,p=parent[i]; while(p>=0){d++;p=parent[p];} depth[i]=d;}
 std::vector<int> outside=outsideTag, dup(R,0);
 for(int i=0;i<R;i++) for(int j=0;j<i;j++)
   if(rooms[i].size()==rooms[j].size() && rooms[i]==rooms[j]){ dup[i]=1; break; }
 printf("{\n\"H\":%d,\"W\":%d,\n\"grid\":[",m.H,m.W);
 for(int r=0;r<m.H;r++){printf("\"");for(int cc=0;cc<m.W;cc++)printf("%c",(cc<(int)m.g[r].size()&&m.g[r][cc]=='.')?'.':'@');printf("\"%s",r==m.H-1?"":",");}
 printf("],\n\"steps\":[\n");
 bool first=true;
 for(int i=0;i<R;i++){
   if(!first)printf(",\n");first=false;
   string col = combined[i]?"phantom":(depth[i]==1?"room":(depth[i]==2?"room2":"phantom"));
   int rccap=(int)(rooms[i].size()+roomCorridor[i].size());
   printf("{\"msg\":\"SUMMARY room %d: %d cells, depth %d, parent %d, cap %d%s\",",
     i,(int)rooms[i].size(),depth[i],parent[i],(int)rooms[i].size(),combined[i]?" [COMBINED]":"");
   printf("\"detail\":\"room cap=%d (counter[%d,%d]), rc cap=%d (counter[%d,%d]). %s\",",
     (int)rooms[i].size(),roomCtr[i].first,roomCtr[i].second,
     rccap,rcCtr[i].first,rcCtr[i].second,
     outside[i]?"OUTSIDE - counters ~dead (huge cap).":"REAL room.");
   printf("\"outside\":%d,\"dup\":%d,",outside[i],dup[i]);
   printf("\"rccap\":%d,",rccap);
   printf("\"roomctr\":[%d,%d],\"rcctr\":[%d,%d],",roomCtr[i].first,roomCtr[i].second,rcCtr[i].first,rcCtr[i].second);
   printf("\"corridor\":["); bool f=1;for(auto&p:roomCorridor[i]){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=0;}printf("],");
   printf("\"color\":\"%s\",\"roomid\":%d,\"cells\":[",col.c_str(),i);
   f=1;for(auto&p:rooms[i]){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=0;}printf("]}");
 }
 printf("\n]}\n");
}
