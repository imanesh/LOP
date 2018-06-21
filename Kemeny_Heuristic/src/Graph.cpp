
#include "Graph.h"

void Graph::insert(Edge e){
	int v = e.v, w = e.w;
	adj[v].push_back(w);
	if (!digraph) { 
		adj[w].push_back(v);
	}
	++nEdge;
}

void Graph::Print(){

}

bool Graph::isPath(int v, int w){
	vector<color> c;
	vector<int> parent;
	c.resize(V());
	parent.resize(V());
	vector<int> d;
	d.resize(V());
	vector<int> f;
	f.resize(V());
	int time =0;
	for(int u= 0; u < V() ; ++u){
		c[u] = white;
		parent[u] = -1;
	}
	stack<int> s;
	parent[w] = w;
	s.push(w);
	while(!s.empty()){
		int u  = s.top();
		c[u] = gray;	
		s.pop();
		if (u == v)
			return true;
		list<int>::iterator it;
		for(it = adj[u].begin(); it != adj[u].end(); ++it)
			if(c[*it] == white){
				parent[*it] = u;
				s.push(*it);
			}
		c[u] = black;
	}
	return false;
}

void Graph::DFS_Visit(vector<color>& c, vector<int>& parent, vector<int>& d, vector<int>& f, int u, int time){
	c[u]= gray;
	++time;
	d[u]=time;
	list<int>::iterator it;
	for(it = adj[u].begin(); it != adj[u].end(); ++it)
		if(c[*it] == white){
			parent[*it] = u;
			DFS_Visit(c,parent,d,f,*it,time);
		}
	c[u] = black;
	++time;
	f[u] = time;
}

void Graph::DFS(){
	vector<color> c;
	vector<int> parent;
	c.resize(V());
	parent.resize(V());
	vector<int> d;
	d.resize(V());
	vector<int> f;
	f.resize(V());
	int time =0;
	for(int u= 0; u < V() ; ++u){
		c[u] = white;
		parent[u] = -1;
	}	
	for(int u =0; u < V(); ++u)
		if(c[u] == white)
			DFS_Visit(c,parent,d,f,u,time);
}
