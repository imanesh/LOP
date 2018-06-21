
#include "adjlist.h"
/*
void AdjList::Initial(int n){
	int l = 0;
	for(int i=0; i < n ; i++)
		for(int j=i+1 ; j < n; j++){
			bool flag = AddEdge(i, j, l);
			if(flag)
				++l;
			flag = AddEdge(j,i,l);
			if(flag)
				++l;
		}
}
*/
bool AdjList::AddEdge (int u, int v, int node)
{
	assert(u != v && v != node && node != u);
	if (PairToList.find(make_pair(u,v)) == PairToList.end()){
		list<int> l ;
		l.push_back(node);
		//cout << "added to ( " << u << "," << v << " ) -> " << node << endl;
		PairToList.insert (make_pair (
				make_pair (u, v),
				l
				)
			);
		return true;
	}
	//if ( PairToList.find(make_pair (u, v)) != PairToList.end())
	else {
		// you find the label out in node and you have to add it to the list
		list<int> l = PairToList.find(make_pair (u, v))->second ;
		if (find(l.begin(), l.end(), node) == l.end()){	
			//cout << "added to ( " << u << "," << v << " ) -> " << node << endl;
			//cout << "list size was = " << l.size()<< endl;
			PairToList.find(make_pair (u, v))->second.push_back(node);
			//cout << "list size after is " << l.size()<< endl;
			//if (find(l.begin(), l.end(), node) != l.end()) 
			//	cout << "list size now is " << l.size()<< endl;
			//else cout << "not added correctly" << endl;
			return true;
		}
	}
	return false;

}

list<int> AdjList::GetList (int u, int v)
{
	list<int> l;
	l.clear();
	if (PairToList.find (make_pair (u, v))!= PairToList.end())
		return PairToList.find (make_pair (u, v))->second;

	if (PairToList.find (make_pair (v, u))!= PairToList.end ())
		return PairToList.find (make_pair (v, u))->second;

	return l;
}

map<pair<int, int>, list<int> >::iterator AdjList::GetIterator ()
{
	return PairToList.begin ();

}

map<pair<int, int>, list<int> >::iterator AdjList::EndIterator ()
{
	return PairToList.end();

}

void AdjList::Print(){
	cout << " Node weight is= \t\t" << node_weight << endl;
	for(map<pair<int,int>,list<int> >::iterator it = GetIterator(); it !=EndIterator(); it++){
		cout << " ( " << it->first.first << " , " << it->first.second << " ) --> [ " ;
		list<int>::iterator l_it;
		for(l_it = it->second.begin(); l_it != it->second.end(); l_it++ )
			cout << *l_it << "  " ;
		cout << " ] " << endl;	
	}
	cout << endl;	
}

