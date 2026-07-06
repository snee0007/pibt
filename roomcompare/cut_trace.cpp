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
 set<pair<int,int>> seent;
 struct Cut{set<pair<int,int>>chain,ra,rb;pair<int,int>a,b,ctrA,ctrB;int aOutside,bOutside;};
 vector<Cut> cuts;
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
       Cut cut; cut.chain=ch; cut.a=a; cut.b=b; cut.ra=ra; cut.rb=rb;
       // outside tag: same rule as nest.cpp
       bool aO=touchesOut(ra), bO=touchesOut(rb);
       int realIsA;
       if(aO&&!bO) realIsA=0;
       else if(bO&&!aO) realIsA=1;
       else realIsA=(ra.size()<=rb.size())?1:0;
       // realIsA=1 means ra is REAL (outside=0), rb is OUTSIDE(outside=1)
       cut.aOutside = realIsA?0:1;   // is side A the outside room?
       cut.bOutside = realIsA?1:0;
       // chain cell next to a point
       auto chainNext=[&](pair<int,int> pt)->pair<int,int>{
         for(auto&y:m.nbr[pt])if(ch.count(y))return y; return pt; };
       // RULE: REAL room -> counter = chain cell next to its point (one cell outside door)
       //       OUTSIDE room -> counter = the point itself (its own edge)
       cut.ctrA = cut.aOutside ? a : chainNext(a);
       cut.ctrB = cut.bOutside ? b : chainNext(b);
       cuts.push_back(cut);
     }
   }
 }
 printf("{\n\"H\":%d,\"W\":%d,\n\"grid\":[",m.H,m.W);
 for(int r=0;r<m.H;r++){printf("\"");for(int cc=0;cc<m.W;cc++)printf("%c",(cc<(int)m.g[r].size()&&m.g[r][cc]=='.')?'.':'@');printf("\"%s",r==m.H-1?"":",");}
 printf("],\n\"cuts\":[\n");
 for(size_t i=0;i<cuts.size();i++){auto&C=cuts[i];
   if(i)printf(",\n");
   printf("{\"cut\":%d,",(int)i+1);
   printf("\"chain\":["); bool f=1;for(auto&p:C.chain){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=0;}printf("],");
   printf("\"a\":[%d,%d],\"b\":[%d,%d],",C.a.first,C.a.second,C.b.first,C.b.second);
   printf("\"aOutside\":%d,\"bOutside\":%d,",C.aOutside,C.bOutside);
   printf("\"counterA\":[%d,%d],\"counterB\":[%d,%d],",C.ctrA.first,C.ctrA.second,C.ctrB.first,C.ctrB.second);
   printf("\"sizeA\":%d,\"sizeB\":%d,",(int)C.ra.size(),(int)C.rb.size());
   printf("\"roomA\":["); f=1;for(auto&p:C.ra){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=0;}printf("],");
   printf("\"roomB\":["); f=1;for(auto&p:C.rb){printf("%s[%d,%d]",f?"":",",p.first,p.second);f=0;}printf("]}");
 }
 printf("\n]}\n");
}
