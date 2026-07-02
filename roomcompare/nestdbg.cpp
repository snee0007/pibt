#include "common.h"
Map m;
int main(int c,char**v){
 m.load(v[1]);
 set<pair<int,int>> thin;
 for(auto&rc:m.cells) if(m.deg(rc)<=2) thin.insert(rc);
 printf("thin cells (deg<=2): %d\n",(int)thin.size());
 auto flood=[&](pair<int,int>s,set<pair<int,int>>&bl){
   set<pair<int,int>>seen; if(bl.count(s))return seen;
   seen.insert(s); vector<pair<int,int>>st={s};
   while(st.size()){auto x=st.back();st.pop_back();
     for(auto&y:m.nbr[x])if(!bl.count(y)&&!seen.count(y)){seen.insert(y);st.push_back(y);}}
   return seen;};
 set<pair<int,int>> seent; int ci=0;
 for(auto&s:thin){
   if(seent.count(s))continue;
   set<pair<int,int>>ch={s}; seent.insert(s); vector<pair<int,int>>st={s};
   while(st.size()){auto x=st.back();st.pop_back();
     for(auto&y:m.nbr[x])if(thin.count(y)&&!ch.count(y)){ch.insert(y);seent.insert(y);st.push_back(y);}}
   set<pair<int,int>>eps;
   for(auto&cc:ch)for(auto&y:m.nbr[cc])if(!thin.count(y))eps.insert(y);
   printf("chain %d: %d cells, %d endpoints",ci,(int)ch.size(),(int)eps.size());
   if(eps.size()==2){
     auto it=eps.begin();auto a=*it++;auto b=*it;
     auto ra=flood(a,ch);
     printf(" -> A reaches B? %s",ra.count(b)?"YES (not room)":"NO (ROOM!)");
     if(!ra.count(b)){auto rb=flood(b,ch);printf(" sealed size=%d",(int)min(ra.size(),rb.size()));}
   }
   printf("\n");ci++;
 }
}
