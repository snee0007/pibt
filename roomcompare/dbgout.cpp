#include "common.h"
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
 auto tout=[&](set<pair<int,int>>&r){for(auto&p:r)for(auto&y:m.nbr[p])
   if(y.first==0||y.second==0||y.first==m.H-1||y.second==m.W-1)return true;return false;};
 set<pair<int,int>> seent;int ci=0;
 for(auto&s:thin){if(seent.count(s))continue;
   set<pair<int,int>>ch={s};seent.insert(s);vector<pair<int,int>>st={s};
   while(st.size()){auto x=st.back();st.pop_back();
     for(auto&y:m.nbr[x])if(thin.count(y)&&!ch.count(y)){ch.insert(y);seent.insert(y);st.push_back(y);}}
   set<pair<int,int>>eps;for(auto&cc:ch)for(auto&y:m.nbr[cc])if(!thin.count(y))eps.insert(y);
   if(eps.size()!=2){ci++;continue;}auto it=eps.begin();auto a=*it++;auto b=*it;auto ra=flood(a,ch);
   if(!ra.count(b)){auto rb=flood(b,ch);
     printf("chain%d: sideA=%d(out=%d) sideB=%d(out=%d)\n",ci,(int)ra.size(),(int)tout(ra),(int)rb.size(),(int)tout(rb));}
   ci++;}
}
