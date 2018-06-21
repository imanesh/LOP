/*
 * AdjList.h
 *
 *  Created on: Mar 11, 2016
 *      Author: eia2
 */

#ifndef GRAPH_H_
#define GRAPH_H_
#include "master_header.h"

using namespace std;

struct Edge{
	int v;
	int w ;
	Edge(int V,int W): v(V), w(W){};
};
class Graph{
private:
	vector<list<int> > adj;
	int nEdge;
	int nVertex;
	bool digraph; 

public:
	Graph(int V, bool digraph = false): adj(V), nVertex(V), nEdge(0), digraph(digraph)
	{adj.resize(V);};
	//~Graph();
	int V() const {return nVertex;}
	int E() const {return nEdge;}
	bool directed() const {return digraph;}
	void insert(Edge e);
	void remove(Edge e);
	bool edge(int v, int w) const;
	bool isPath(int v , int w);
	void DFS_Visit(vector<color>& c, vector<int>& parent, vector<int>& d, vector<int>& f, int u, int time);
	void DFS();
	void BFS();
	list<int> GetNeighbor(int v);
	void Print();
	
	class adjeIterator;
	friend class adjIterator;
	};

#endif /* GRAPH_H_ */
