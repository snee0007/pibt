#include <bits/stdc++.h>
using namespace std;
struct Map {
  int H,W; vector<string> g;
  bool opn(int r,int c){return r>=0&&r<H&&c>=0&&c<(int)g[r].size()&&g[r][c]=='.';}
  vector<pair<int,int>> cells;
  map<pair<int,int>,vector<pair<int,int>>> nbr;
  void load(string p){
    ifstream f(p); string l; vector<string> raw;
    while(getline(f,l)){ if(!l.empty()&&l.back()=='\r')l.pop_back(); raw.push_back(l); }
    int mi=0;
    for(int i=0;i<(int)raw.size();i++){ if(raw[i].rfind("map",0)==0){mi=i+1;break;}
      if(raw[i].rfind("height",0)==0)H=stoi(raw[i].substr(7));
      if(raw[i].rfind("width",0)==0)W=stoi(raw[i].substr(6)); }
    for(int i=mi;i<(int)raw.size();i++) if(!raw[i].empty()) g.push_back(raw[i]);
    H=g.size(); W=0; for(auto&s:g) W=max(W,(int)s.size());
    int dr[]={-1,1,0,0},dc[]={0,0,-1,1};
    for(int r=0;r<H;r++)for(int c=0;c<(int)g[r].size();c++)if(opn(r,c))cells.push_back({r,c});
    for(auto&rc:cells){vector<pair<int,int>>n;
      for(int k=0;k<4;k++)if(opn(rc.first+dr[k],rc.second+dc[k]))n.push_back({rc.first+dr[k],rc.second+dc[k]});
      nbr[rc]=n;}
  }
  int deg(pair<int,int>rc){return nbr[rc].size();}
};
