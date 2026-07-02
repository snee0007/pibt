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
 vector<set<pair<int,int>>> rooms; set<pair<int,int>> seent;
 for(auto&s:thin){if(seent.count(s))continue;
   set<pair<int,int>>ch={s};seent.insert(s);vector<pair<int,int>>st={s};
   while(st.size()){auto x=st.back();st.pop_back();
     for(auto&y:m.nbr[x])if(thin.count(y)&&!ch.count(y)){ch.insert(y);seent.insert(y);st.push_back(y);}}
   set<pair<int,int>>eps;for(auto&cc:ch)for(auto&y:m.nbr[cc])if(!thin.count(y))eps.insert(y);
   if(eps.size()!=2)continue;auto it=eps.begin();auto a=*it++;auto b=*it;auto ra=flood(a,ch);
   if(!ra.count(b)){auto rb=flood(b,ch);bool ao=tout(ra),bo=tout(rb);
     set<pair<int,int>> rs; if(ao&&!bo)rs=rb;else if(bo&&!ao)rs=ra;else rs=(ra.size()<=rb.size())?ra:rb;
     rooms.push_back(rs);}}
 int R=rooms.size();
 vector<bool> combined(R,false);
 for(int j=0;j<R;j++){ vector<int> cont;
   for(int i=0;i<R;i++){ if(i==j||rooms[j].size()<=rooms[i].size())continue;
     bool ct=true; for(auto&p:rooms[i]) if(!rooms[j].count(p)){ct=false;break;} if(ct)cont.push_back(i);}
   for(size_t a=0;a<cont.size()&&!combined[j];a++)for(size_t b=a+1;b<cont.size()&&!combined[j];b++){
     bool ov=false; for(auto&p:rooms[cont[a]]) if(rooms[cont[b]].count(p)){ov=true;break;}
     if(!ov)combined[j]=true; } }
 for(int i=0;i<R;i++)printf("room %d (%d cells) combined=%d\n",i,(int)rooms[i].size(),(int)combined[i]);
}
