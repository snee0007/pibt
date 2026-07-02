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
 // ANDY chain method -> roomsets
 vector<set<pair<int,int>>> rooms;
 std::vector<int> outsideTag;
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
       // DOUBLE-SIDED: keep BOTH sides. Tag using OLD single-sided picker:
       // the side the old rule would KEEP is "real" (outside=0), the other is "outside".
       bool aO=touchesOut(ra), bO=touchesOut(rb);
       int realIsA;  // 1 = ra is the real room, 0 = rb is
       if(aO&&!bO) realIsA=0;        // ra touches outside -> rb is real
       else if(bO&&!aO) realIsA=1;   // rb touches outside -> ra is real
       else realIsA=(ra.size()<=rb.size())?1:0;  // neither/both -> smaller is real
       if(realIsA){ rooms.push_back(ra); outsideTag.push_back(0);
                    rooms.push_back(rb); outsideTag.push_back(1); }
       else { rooms.push_back(rb); outsideTag.push_back(0);
              rooms.push_back(ra); outsideTag.push_back(1); }}
   }
 }
 int R=rooms.size();
 // exits[i] = cells just outside room i
 vector<set<pair<int,int>>> exits(R);
 for(int i=0;i<R;i++)
   for(auto&cid:rooms[i])
     for(auto&y:m.nbr[cid]) if(!rooms[i].count(y)) exits[i].insert(y);
 // TRUE parent: j contains all of i's cells AND all of i's exits land inside j
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
 // TAGS for visualization filtering (data kept either way)
 std::vector<int> outside=outsideTag, dup(R,0);
 for(int i=0;i<R;i++) for(int j=0;j<i;j++)
   if(rooms[i].size()==rooms[j].size() && rooms[i]==rooms[j]){ dup[i]=1; break; }
 // JSON
 printf("{\n\"H\":%d,\"W\":%d,\n\"grid\":[",m.H,m.W);
 for(int r=0;r<m.H;r++){printf("\"");for(int cc=0;cc<m.W;cc++)printf("%c",(cc<(int)m.g[r].size()&&m.g[r][cc]=='.')?'.':'@');printf("\"%s",r==m.H-1?"":",");}
 printf("],\n\"steps\":[\n");
 bool first=true;
 for(int i=0;i<R;i++){
   if(!first)printf(",\n");first=false;
   string col = combined[i]?"phantom":(depth[i]==1?"room":(depth[i]==2?"room2":"phantom"));
   const char* kind = combined[i]?"COMBINED":"room";
   printf("{\"msg\":\"SUMMARY room %d: %d cells, depth %d, parent %d, cap %d%s\",",
     i,(int)rooms[i].size(),depth[i],parent[i],(int)rooms[i].size(),combined[i]?" [COMBINED]":"");
   printf("\"detail\":\"%s. depth=%d, parent=%d, cap=%d. %s\",",
     kind,depth[i],parent[i],(int)rooms[i].size(),
     combined[i]?"Observation counter - counts but never blocks entry.":"Normal room - gates entry when full.");
   printf("\"outside\":%d,\"dup\":%d,",outside[i],dup[i]);
   printf("\"color\":\"%s\",\"roomid\":%d,\"cells\":[",col.c_str(),i);
   bool f=true;for(auto&p:rooms[i]){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=false;}printf("]}");
 }
 printf("\n]}\n");
}
