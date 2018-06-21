/*
 * AdjList.h
 *
 *  Created on: Mar 11, 2016
 *      Author: eia2
 */

#ifndef ADJLIST_H_
#define ADJLIST_H_
#include "master_header.h"

using namespace std;
class AdjList{
private:
	map<pair<int, int>, list<int> > PairToList;
	double node_weight;

public:
	//AdjList();
	//~AdjList();
	void Initial(int n);
    list<int> GetList (int u, int v);
    //pair<int, int> GetEdgeByLabel (int label);
    map<pair<int, int>, list<int> >::iterator GetIterator ();
    map<pair<int, int>, list<int> >::iterator EndIterator ();
	list<int>::iterator GetListIterator(pair<int,int> p);
	list<int>::iterator EndListIterator(pair<int,int> p);
    bool AddEdge(int v, int w, int node);
	int fan_out_size() const {return PairToList.size();}; // size of PaitToList
	void setNodeWeight(double w) { node_weight = w; };
	double getNodeWeight() const {return node_weight;};
	void Print();
};

#endif /* EDGEMANAGER_H_ */
