#include "common.h"
Map m;
int main(int c,char**v){
 m.load(v[1]);
 set<pair<int,int>> thin;
 for(auto&rc:m.cells) if(m.deg(rc)<=2) thin.insert(rc);
 set<pair<int,int>> seent;int ci=0;
 for(auto&s:thin){if(seent.count(s))continue;
   set<pair<int,int>>ch={s};seent.insert(s);vector<pair<int,int>>st={s};
   while(st.size()){auto x=st.back();st.pop_back();
     for(auto&y:m.nbr[x])if(thin.count(y)&&!ch.count(y)){ch.insert(y);seent.insert(y);st.push_back(y);}}
   set<pair<int,int>>eps;for(auto&cc:ch)for(auto&y:m.nbr[cc])if(!thin.count(y))eps.insert(y);
   printf("chain %d: %d cells, %d eps",ci,(int)ch.size(),(int)eps.size());
   for(auto&p:ch)printf(" (%d,%d)",p.first,p.second);
   printf(" | eps:");for(auto&e:eps)printf(" (%d,%d)",e.first,e.second);
   printf("\n");ci++;}
}
