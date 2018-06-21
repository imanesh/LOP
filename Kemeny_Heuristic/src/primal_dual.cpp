
/*
 * heuristic_solver.cpp
 *  Created on: Jan 14, 2014
 *  Author: eia2
 */
#include "primal_dual.h"
#include "heuristic_solver.h"
#define RC_EPS 1.0e-10

struct trpl{
	int i,j,k;
	double value;
	trpl(int w,int s,int t, double v):i(w),j(s),k(t),value(v){}
	bool operator <(trpl a) const{
		if (value < a.value)
			return true;
		else return false;	
	}
};


void PrimalDualSolver::InitialConstraintManager(vector<vector<dicycle> >& ConstManager, EdgeManager& Edge, CycleManager& Cycle, int size_edge){
	for(int i=0; i < size_edge; i++ ){
		pair<int,int> e = Edge.GetEdgeByLabel(i);
		int temp[3];
		temp[0]= e.first;
		temp[1] = e.second;
		for (int k = 0; k < R.Candidates(); k++)
			if(k != temp[0] && k!=temp[1]){
				temp[2]	= k;
				int j =0;
				while (j < 3){
					if((temp[j%3] < temp[(j+1)%3] && temp[(j+1)%3] < temp[(j+2)%3]) ||
							(temp[j%3] > temp[(j+1)%3] && temp[(j+1)%3] > temp[(j+2)%3])){
						dicycle d(temp[j%3],temp[(j+1)%3],temp[(j+2)%3]);
						ConstManager[i].push_back(d);
						break;
					} else j++;
				}
			}
	}
}

void PrimalDualSolver:: PrintConstManager(vector<vector<dicycle> >& ConstManager,EdgeManager& Edge){
	vector<vector< dicycle> >::const_iterator it = ConstManager.begin();
	int i =0;
	for( it = ConstManager.begin(); it!=ConstManager.end(); it++){
		pair<int,int> e = Edge.GetEdgeByLabel(i++);
		cout << "y_( " << e.first << " , " << e.second << " )  " ;
		for(vector<dicycle>::const_iterator it_j = it->begin(); it_j != it->end(); it_j++ ){
			cout << "+ z_( " << it_j->i << " , " << it_j->j << " , " << it_j->k << " )  " ;
		}
		cout << " <= 0 " << endl;
	}
}

double PrimalDualSolver::ComputeTeta(vector<vector<dicycle> >& ConstManager, dual_sol_vectors& dual_solution, 
		dual_sol_vectors& drp_solution, EdgeManager& Edge, CycleManager& Cycle){
	double teta = 100000000;;
	// y_ij + sum_{k}z_ijk <= c_ji constraints
	cout << "#================ Teta Computation ===========#" << endl;
	int cm_size = ConstManager.size();
	//cout << setfill(' ') << setw(5);
	cout.precision(3);
	cout << "----------------------------------------------------------------------" << endl;
	cout << setw(7) << "Pair " << setw(8) << " LHS " << setw(14) << "dual_sol"<< setw(10) << "drp_sol" 
		<< setw(11) <<  "Teta_coef" << setw(8) << "w_ij" << setw(11) << "teta" << endl;	
	cout << "----------------------------------------------------------------------" << endl;
	for(int i =0; i < cm_size; i++){
		pair<int,int> e = Edge.GetEdgeByLabel(i);
		cout << setw(3)<< "(" << e.first << "," << e.second <<")" << setw(8);
		double LHS = dual_solution.y[i];
		double teta_coef = drp_solution.y[i];
		vector<dicycle>::const_iterator it;
		for(it = ConstManager[i].begin(); it != ConstManager[i].end(); it++){
			int label_cycle = Cycle.GetLabel(it->i, it->j, it->k);
			LHS += dual_solution.z[label_cycle];
			teta_coef += drp_solution.z[label_cycle];
		}
		cout <<  LHS << setw(10) << dual_solution.y[i] << setw(10) << drp_solution.y[i] << setw(11) 
			<<  teta_coef << setw(10) << R.Get_m_Array()[e.second][e.first] 
			<< setw(13) << static_cast<double>((R.Get_m_Array()[e.second][e.first]-LHS)/teta_coef) << endl; 
		if (teta_coef > 0)
			if ((R.Get_m_Array()[e.second][e.first]-LHS)/teta_coef < teta ) // need to be done carfeully
				// if the teta_coef is minus then inseatd of upper bund on theta we have lower bound ???
				teta = static_cast<double>((R.Get_m_Array()[e.second][e.first]-LHS)/teta_coef); 
	}
	cout << "----------------------------------------------------------------------" << endl;
	cout << endl;
	cout << "-------------------------------------------------" << endl;
	cout << "dicycle" << setw(13) << "dual_sol" << setw(10) << "drp_sol" << setw(10) << "teta" << endl;	
	cout << "-------------------------------------------------" << endl;
	//z_ijk >= 0 constraints
	for(int i =0; i < dual_solution.z.size(); i++){
		cycle c = Cycle.GetCycleByLabel(i) ;
		cout << "(" << c.first.first << "," << c.first.second << "," << c.second << ")" << setw(11);
		cout <<  dual_solution.z[i] << setw(10) << drp_solution.z[i] << setw(13)
			<< static_cast<double>(dual_solution.z[i]/drp_solution.z[i])  << endl; 
		if (drp_solution.z[i] < 0)
			if (dual_solution.z[i]/abs(drp_solution.z[i]) < teta)
				teta =static_cast<double>(dual_solution.z[i]/abs(drp_solution.z[i]));
	}
	cout << "-------------------------------------------------" << endl;
	cout << "teta is = " << teta << endl << endl; 
	return teta;
}

bool PrimalDualSolver::DualFeasibile(vector<vector<dicycle> >& ConstManager, dual_sol_vectors& dual_solution, 
		EdgeManager& Edge, CycleManager& Cycle, vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycles)
{
	int cm_size = ConstManager.size();
	for(int i =0; i < cm_size; i++){
		pair<int,int> e = Edge.GetEdgeByLabel(i);
		double LHS = dual_solution.y[i];
		vector<dicycle>::const_iterator it;
		for(it = ConstManager[i].begin(); it != ConstManager[i].end(); it++){
			int label_cycle = Cycle.GetLabel(it->i, it->j, it->k);
			LHS += dual_solution.z[label_cycle];
		}
		if (LHS -  R.Get_m_Array()[e.second][e.first] > RC_EPS ){ 
			cout << "The pair related to infeasibility is = (" << e.first << "," << e.second << ")" << endl; 
			cout << "LHS = " << LHS << " and c_ji is = " << R.Get_m_Array()[e.second][e.first]<< endl;
			return false;
		}
		else if(LHS == R.Get_m_Array()[e.second][e.first]) 
			tight_edges.push_back(e);
	}
	for(int i =0; i < dual_solution.z.size(); i++){
		cycle c = Cycle.GetCycleByLabel(i);
		dicycle d(c.first.first,c.first.second,c.second);
		if (dual_solution.z[i] < 0){ 
			cout << "dual z_i is " << dual_solution.z[i] << endl;
			return false;
		}
		else if (dual_solution.z[i] == 0) 
			tight_cycles.push_back(d);
	}
	return true;
}





void PrimalDualSolver::PrintVector(vector<double>& s)
{
	int counter = 0;
	vector<double>::const_iterator i;
	for(i = s.begin(); i != s.end(); i = i+2){
		cout << *i << "  ";  // increase by 1 in order to match the convention indexing scheme
	}
}



double PrimalDualSolver::SolutionValue(dual_sol_vectors& v){
	double obj_val_drp = 0;
	for(int i=0; i<v.y.size(); i=i+2)
		obj_val_drp += v.y[i];
	for(int i= 0; i<v.z.size(); i++)
		obj_val_drp += v.z[i];
	return obj_val_drp;
}

bool BothInTightEdgeSet(vector<pair<int,int> >& tight_edges, pair<int,int> edge){
	bool flag_edge = false, flag_reverse_edge = false;
	vector<pair<int,int> >::iterator it;
	for(it = tight_edges.begin(); it != tight_edges.end(); it++){
		if((!flag_edge) && (it->first == edge.first) && (it->second == edge.second))
			flag_edge = true;
		if((!flag_reverse_edge) && (it->first == edge.second) && (it->second == edge.first))
			flag_reverse_edge = true;
		if (flag_edge && flag_reverse_edge)
			return true;
	}
	return flag_edge && flag_reverse_edge;
}

bool BothInTightEdgeSet(vector<pair<int,int> >& tight_edges, int i , int j){
	bool flag_edge = false, flag_reverse_edge = false;
	vector<pair<int,int> >::iterator it;
	for(it = tight_edges.begin(); it != tight_edges.end(); it++){
		if((!flag_edge) && (it->first == i) && (it->second == j))
			flag_edge = true;
		if((!flag_reverse_edge) && (it->first == j) && (it->second == i))
			flag_reverse_edge = true;
		if (flag_edge && flag_reverse_edge)
			break;
	}
	return flag_edge && flag_reverse_edge;

}



bool InTightEdgeSet(vector<pair<int,int> >& tight_edges,int i,int j){
	vector<pair<int,int> >::iterator it;
	for(it = tight_edges.begin(); it != tight_edges.end(); it++){
		if( (it->first == i) && (it->second == j))
			return true;
	}
	return false;
}


vector<AdjList> PrimalDualSolver::CreateGraph(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycles, 
		EdgeManager& Edge, CycleManager& Cycle, dual_sol_vectors& drp_solution, vector<int>& nTightEdgeInCycle, vector<int>& nReverseTightEdgeInCycle){
	//graph data structure creation and initialization
	vector<AdjList> Graph;
	Graph.resize(Cycle.CycleCount());
	vector<int> node_weight_tracker;
	node_weight_tracker.resize(Cycle.CycleCount());
	for(int i = 0; i < Cycle.CycleCount(); i++){
		Graph[i].setNodeWeight(0);
		//node_weight_tracker[i] = 0;
	}

	//making graph connections
	cout << "cycle number is : " << Cycle.CycleCount()<< endl;
	for(int i = 0; i < Cycle.CycleCount(); i++){	
		//cout << "cycle number is : "<< i  << endl;
		cycle c = Cycle.GetCycleByLabel(i);
		//cout << "helleo" <<  endl;
		int e[3];
		e[0] = c.first.first;
		e[1] = c.first.second;
		e[2] = c.second;

		//print info of the cycle
		cout << endl <<"cycle number is " << i << endl;
		cout << "cycle vertices are = " << e[0] << "," << e[1] << "," << e[2] << endl;
		cout << "tight edges in this cycle are = " ;
		//calculate the number of tight edges for the cycle i
		for(int j = 0; j < 3 ; j++){
			if(InTightEdgeSet(tight_edges, e[j %3], e[(j+1)%3]))
				nTightEdgeInCycle[i]++;
			//if(InTightEdgeSet(tight_edges, e[(j+1)%3],e[j %3]))
			//	nReverseTightEdgeInCycle[i]++;
			if(BothInTightEdgeSet(tight_edges,e[j % 3],e[(j+1)%3])){
				nReverseTightEdgeInCycle[i]++;
				cout << "(" << e[j%3] << "," << e[(j+1)%3] << ")  " ;
			}
			//node_weight_tracker[i]++;
		}
		cout << endl <<"connected cycles are : " << endl;
		//create the connection for node i
		for(int j = 0; j < 3 ; j++){
			//cout << "j is :" << j << endl;
			pair<int,int> edge = make_pair(e[j%3],e[(j+1)%3]);
			if(BothInTightEdgeSet(tight_edges, edge))
				for(int k = 0; k < R.Candidates(); k++){
					int temp[3];
					temp[0] = edge.second;
					temp[1] = edge.first;
					temp[2] = k;
					int t =0;
					while (t < 3)
						if((temp[t%3] < temp[(t+1)%3] && temp[(t+1)%3] < temp[(t+2)%3]) ||
								(temp[t%3] > temp[(t+1)%3] && temp[(t+1)%3] > temp[(t+2)%3])){
							//get the label of the neighbor cycle
							int n = Cycle.GetLabel(temp[t%3],temp[(t+1)%3],temp[(t+2)%3]); 
							//add the connection in graph cycle
							Graph[i].AddEdge(e[j%3],e[(j+1)%3],n);   //(e[j%3],e[(j+1)%3]) is in i and its reverse is in n
							cout << " " << n << "  " ;	
							break;
						} else t++;
				}
		}
		cout << endl<< endl;
	}	
	cout << "n tight edges" << nTightEdgeInCycle << endl; 
	cout << "n reverese tight edges" << nReverseTightEdgeInCycle << endl; 
	//cout<< "ehsan " << endl;
	// set the node weights
	for(int i = 0; i < Cycle.CycleCount(); i++){
		cout << "cycle is " << i << " and node tracker is " << node_weight_tracker[i] << endl; 
		assert(nTightEdgeInCycle[i] >=0 && nTightEdgeInCycle[i] <= 3);
		Graph[i].setNodeWeight(max(0,nReverseTightEdgeInCycle[i]-1)); // node weight needs to be revisited
		//Graph[i].setNodeWeight(max(0,node_weight_tracker[i]-1));
	}
	//cout << "ehsan 2" << endl;	
	return Graph;
}


int FindMax(vector<int>& n){
	//return the max value index in vector n
	return 1;
}

int FindMax2(vector<int>& n, vector<int>& r,vector<color>& vertex_status){
	// return a cycle which has exactly 2 tight edges and 1 reverse edge tight
	//o.w return -1
	int a ;
	int size = n.size();
	for(int i =0; i < size; ++i){
		//		cout << "n is " << n[i] << " r is " << r[i] << " color is " << vertex_status[i] << endl;
		if(n[i] == 2 && r[i] == 1 && vertex_status[i] == white ){
			//cout << "found the vertex"<< endl;
			return i;
		}
	}
	return -1;
}

bool AllVertexVisited(vector<color>& vertex_status){
	vector<color>::iterator it;
	for(it = vertex_status.begin();it!=vertex_status.end();it++)
		if(*it != black)
			return false;
	return true;
}





solution PrimalDualSolver::Test_CombinatorialDRP_Random_Input_Graph(){

	R.SetCandidates(6);
	cout << "r candidayes is " << R.Candidates() << endl;
	solution s(R.Candidates());
	EdgeManager Edge;
	CycleManager Cycle;
	Edge.Initial(R.Candidates());
	Edge.Print();
	Cycle.Initial(R.Candidates());
	Cycle.Print();
	vector<vector<dicycle> > ConstManager;
	int size_edge = R.Candidates() * (R.Candidates()-1);
	ConstManager.resize(size_edge);
	InitialConstraintManager(ConstManager, Edge, Cycle,size_edge);
	// PrintConstManager(ConstManager, Edge);
	dual_sol_vectors dual_solution(R.Candidates());
	dual_sol_vectors drp_solution(R.Candidates());
	vector<pair<int,int> > tight_edges;
	vector<dicycle> tight_cycles;

	//generate random tight edges and cycles
	Generate_Random_Tight_Edge_Cycle(tight_edges,tight_cycles);
	cout << "tight edges are : " << tight_edges << endl;
	//solve by the combinatorial DRP
	CombinatorialDRPSolver(tight_edges,tight_cycles,Edge,Cycle,drp_solution);
	SolveDRP(tight_edges,tight_cycles,Edge,Cycle,drp_solution);
	return s;
}

void PrimalDualSolver::Generate_Random_Tight_Edge_Cycle(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycles){
/*
	for(int i = 0; i < R.Candidates(); ++i)
		for(int j = 0; j < R.Candidates(); ++j)
			if(i != j ){
				double r = ((double)rand() / (RAND_MAX));
				if( r > 0.5)
					tight_edges.push_back(make_pair(i,j));	   
			}
*/
	//tight_edges.push_back(make_pair(0,1));	
	//tight_edges.push_back(make_pair(1,2));
	//tight_edges.push_back(make_pair(2,1));
//	tight_edges.push_back(make_pair(3,2));
	//tight_edges.push_back(make_pair(2,0));
	//tight_edges.push_back(make_pair(1,4));	
//	tight_edges.push_back(make_pair(0,2));
//	tight_edges.push_back(make_pair(3,0));
//	tight_edges.push_back(make_pair(3,5));
//	tight_edges.push_back(make_pair(5,1));
//	tight_edges.push_back(make_pair(5,3));
//	tight_edges.push_back(make_pair(5,4));
//	tight_edges.push_back(make_pair(4,5));
//	tight_edges.push_back(make_pair(1,5));
//	tight_edges.push_back(make_pair(0,3));

	//tight_edges.push_back(make_pair(3,4));   
	//tight_edges.push_back(make_pair(4,1));
	//tight_edges.push_back(make_pair(1,4));
	//tight_edges.push_back(make_pair(4,3));
	//tight_edges.push_back(make_pair(3,4));


	
//	   tight_edges.push_back(make_pair(0,1));	
	   tight_edges.push_back(make_pair(1,2));
	   tight_edges.push_back(make_pair(2,1));
//	   tight_edges.push_back(make_pair(3,2));
	  // tight_edges.push_back(make_pair(2,0));
	   //tight_edges.push_back(make_pair(0,2));
	   //tight_edges.push_back(make_pair(3,0));
//	   tight_edges.push_back(make_pair(3,4));
	   //tight_edges.push_back(make_pair(4,0));
	   //tight_edges.push_back(make_pair(4,1));
//	   tight_edges.push_back(make_pair(2,4));
//	   tight_edges.push_back(make_pair(4,2));

	   /*
	   tight_edges.push_back(make_pair(0,3));	
	   tight_edges.push_back(make_pair(1,2));
	   tight_edges.push_back(make_pair(3,1));
	   tight_edges.push_back(make_pair(0,1));
	   tight_edges.push_back(make_pair(3,0)); */
	   //tight_edges.push_back(make_pair(0,2));
	  // tight_edges.push_back(make_pair(2,1));

}

bool isTerminal(vector<int>& nTightEdgeInCycle, vector<int>& nReverseTightEdgeInCycle,int u){
	if(nTightEdgeInCycle[u] == 1 && nReverseTightEdgeInCycle[u] == 1)
		return true;
	return false;
}


bool isValidMiddleNode(vector<int>& nTightEdgeInCycle,vector<int>& nReverseTightEdgeInCycle,int u){
	if(nTightEdgeInCycle[u] ==2 && nReverseTightEdgeInCycle[u]==2)
		return true;
	return false;
}

void PrimalDualSolver::CombinatorialDRPSolver(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycles, 
		EdgeManager& Edge, CycleManager& Cycle, dual_sol_vectors& drp_solution){

	vector<int> nTightEdgeInCycle, nReverseTightEdgeInCycle;
	nTightEdgeInCycle.resize(Cycle.CycleCount());
	nReverseTightEdgeInCycle.resize(Cycle.CycleCount());
	for(int i=0;i<Cycle.CycleCount();i++){	
		nTightEdgeInCycle[i] = 0;
		nReverseTightEdgeInCycle[i] = 0;
	}
	//vector<vector<int> > Graph = CreateGraphCycle(tight_edges, tight_cycles, Edge, Cycle,drp_solution,nTightEdgeInCycle);
	vector<AdjList> graph = CreateGraph(tight_edges, tight_cycles,Edge,Cycle,drp_solution,nTightEdgeInCycle,nReverseTightEdgeInCycle);

	//printing cycles info
	for(int i = 0; i < graph.size(); ++i){
		cout << "-------------------------------------" << endl;
		cout << " cycle info " << endl;
		cout << " number= \t\t\t" << i << endl;
		cout << " n tight edges= \t\t" << nTightEdgeInCycle[i] << endl; 
		cout << " n reverse tight edges= \t" << nReverseTightEdgeInCycle[i] << endl;
		cycle c = Cycle.GetCycleByLabel(i) ;
		cout << " Cycle's arcs are=   \t"  << c << endl;
		//cout << "helleo" <<  endl;
		int e[3];
		e[0] = c.first.first;
		e[1] = c.first.second;
		e[2] = c.second;
		cout<< "tight edges for this cycle are= " << endl;
		//calculate the number of tight edges for the cycle i
		for(int j = 0; j < 3 ; j++){
			if(BothInTightEdgeSet(tight_edges, e[j %3], e[(j+1)%3]))	
				cout << "( " << e[j%3] << "," << e[(j+1)%3] << " ) and it reverese : ( " << e[(j+1)%3] << "," << e[j%3] << " ) " <<endl; 
			else if(InTightEdgeSet(tight_edges, e[j%3],e[(j+1) %3]))
				cout <<  "( " << e[j%3] << "," << e[(j+1)%3] << " ) " << endl ; 	
		}
		graph[i].Print();
		cout << "-------------------------------------" << endl;
	}
	cout << "ehsan " << endl;

	//increase non-tight edges by one only if their reverse is also not tight
	for(int i=0;i<R.Candidates();i++)
		for(int j=i+1;j<R.Candidates();j++)
			if(!BothInTightEdgeSet(tight_edges,i,j)){
				drp_solution.y[Edge.GetLabel(i,j)] = 1;
				drp_solution.y[Edge.GetLabel(j,i)] = 1;
			}
	//increase those cycles which does not have any tight edges 
	for(int i= 0;i<Cycle.CycleCount();i++)
		if(nTightEdgeInCycle[i]==0)
			drp_solution.z[i] =1;
	vector<color> vertex_status;
	vertex_status.resize(Cycle.CycleCount());

	//run BFS , the grapg might have diffrent component, so this piece of code should be wraped in anothe code to handle 
	//diffrent components
	//only for the begining, might change later!!! make cycles with 3 edge tight out the picture
	for(int i=0; i<Cycle.CycleCount();i++)
		if (nTightEdgeInCycle[i] == 3){
			cout << "another black is = " << i << endl;
			vertex_status[i] = black;
		}
		else
			vertex_status[i] = white;

	//start by finding a cycle with 2 tight and 1 reverse tight edge, end by the same cycle node
	queue<int> Q;
	while(!AllVertexVisited(vertex_status)){	
		int root = FindMax2(nTightEdgeInCycle,nReverseTightEdgeInCycle,vertex_status);
		if (root == -1){
			cout << "There is no such a in creasing path !!!" << endl;
			break;
		}
		cout << "------------------------------------" << endl;
		cout << "Selected root for this itrartion is " << root << endl;
		vertex_status[root] = gray;	
		Q.push(root);
		while(!Q.empty()){
			int u = Q.front();
			Q.pop();
			cout << "next veretx is " << u << "  "<< endl;
			cycle c = Cycle.GetCycleByLabel(u);

			//find the tight edges of the cycle u
			vector<pair<int,int> > tight_edges_u;
			if (InTightEdgeSet(tight_edges,c.first.first,c.first.second))
				tight_edges_u.push_back(make_pair(c.first.first, c.first.second));
			if (InTightEdgeSet(tight_edges,c.first.second,c.second))
				tight_edges_u.push_back(make_pair(c.first.second, c.second));
			if (InTightEdgeSet(tight_edges,c.second,c.first.first))
				tight_edges_u.push_back(make_pair(c.second, c.first.first));

			if (isTerminal(nTightEdgeInCycle,nReverseTightEdgeInCycle,u)){
				//increase y, decrease z , and stop
				vector<pair<int,int> >::iterator it;
				for(it = tight_edges_u.begin(); it != tight_edges_u.end(); it++){	
					drp_solution.y[Edge.GetLabel(it->first,it->second)] = 1;
					drp_solution.y[Edge.GetLabel(it->second,it->first)] = 1;
				}
				drp_solution.z[u] = -1;
				vertex_status[u] = black;
				break;
			}
			else if (isValidMiddleNode(nTightEdgeInCycle,nReverseTightEdgeInCycle,u)){
				// increase y, decrease z and  continue
				vector<pair<int,int> >::iterator it;
				for(it = tight_edges_u.begin(); it != tight_edges_u.end(); it++){	
					drp_solution.y[Edge.GetLabel(it->first,it->second)] = 1;
					drp_solution.y[Edge.GetLabel(it->second,it->first)] = 1;
				}
				drp_solution.z[u] = -1;
			}
			// add neighbor of u to the queue
			vector<pair<int,int> >::iterator it;
			for(it = tight_edges_u.begin(); it != tight_edges_u.end(); it++){
				list<int> l = graph[u].GetList(it->first,it->second);
				if (!l.empty()){
					list<int>::iterator it_l;
					for(it_l = l.begin(); it_l != l.end(); it_l++ ){
						cout << "next to root is : " << *it_l << " and the stat is " << vertex_status[*it_l] << endl;
						if(vertex_status[*it_l] == white){
							vertex_status[*it_l] =gray;
							Q.push(*it_l);
						}
					}
				}
			}	
			//mark u as a visisted node
			vertex_status[u] = black;

		}
		cout << "-----------------------------------" << endl;
		// either find another root or report all the vertex are visited
	}

	double obj_value=0;
	vector<double>::const_iterator it;
	for(it = drp_solution.y.begin(); it != drp_solution.y.end(); ++it)
		obj_value += *it;
	obj_value /=2;	
	for(it = drp_solution.z.begin(); it != drp_solution.z.end(); ++it)
		obj_value += *it;
	cout << "The Combinatorial drp objective function value is = " << obj_value << endl;	
	cout << "drp solution vector  y = { " << drp_solution.y << " }" << endl;
	cout << "drp solution vector z = { " << drp_solution.z << " }" << endl;
	//cmpute the objective finction

}



void PrimalDualSolver::SolveDRP(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycles, 
		EdgeManager& Edge, CycleManager& Cycle, dual_sol_vectors& drp_solution){
	solution s(R.Candidates());
	static int iter= 0;
	try{
		IloEnv env;
		IloModel model(env);
		IloCplex cplex(model);
		NumVarMatrix y(env, R.Candidates());
		for(int i = 0; i < R.Candidates(); i++)
			y[i] = IloNumVarArray(env, R.Candidates(),-IloInfinity,1.0,ILOFLOAT);
		for(int i =0; i < R.Candidates(); i++)
			for(int j =0; j < R.Candidates(); j++){
				stringstream s;
				s << i << "," << j;
				string name("y_");
				name += s.str();
				const char* n_y = name.c_str();	
				y[i][j].setName(n_y);
				//delete n_y;
			}
		ThreeDMatrix z(env,R.Candidates());
		for (int i = 0; i < R.Candidates(); i++){
			z[i] = NumVarMatrix(env,R.Candidates());
			for(int j =0; j < R.Candidates(); j++)
				z[i][j]= IloNumVarArray(env,R.Candidates(),-IloInfinity,1.0,ILOFLOAT);

		}
		for(int i =0; i < R.Candidates(); i++)
			for(int j =0; j < R.Candidates(); j++)
				for(int k =0; k < R.Candidates(); k++){
					stringstream s;
					s << i << "," << j << ","<<  k;
					string name("z_");
					name += s.str();
					const char* n_z = name.c_str();	
					z[i][j][k].setName(n_z);
					//delete n_z;
				}
		//generate the obj_function
		IloExpr obj_fun(env);
		for(int i = 0; i < R.Candidates(); i++)
			for(int j = i+1; j < R.Candidates(); j++)
				obj_fun +=  y[i][j];
		for(int i =0; i < R.Candidates(); i++)
			for(int j = 0; j < R.Candidates(); j++)
				for(int k = 0; k < R.Candidates(); k++)
					if (i!=j && j!=k && k!=i)
						if (!( (j > i && j >k) || (j < i && j < k)) )
							obj_fun += z[i][j][k];
		model.add(IloMaximize(env, obj_fun));
		for(int i =0; i < R.Candidates(); i++)
			for(int j = i+1; j < R.Candidates(); j++)
				model.add(y[i][j] - y[j][i] == 0);
		for(int i = 0; i < R.Candidates(); i++)
			for(int j =0; j < R.Candidates(); j++)
				for(int k = 0; k < R.Candidates(); k++)
					if((j  >= i && j >= k) || ( j < i && j < k ))
						model.add(z[i][j][k] == 0);
		for(int i = 0; i < R.Candidates(); i++)
			model.add(y[i][i] == 0);
		//add tight dual constraints in here
		//add tight cycle constraints
		for(vector<dicycle>::iterator it = tight_cycles.begin(); it != tight_cycles.end(); it++ ){
			model.add(z[it->i][it->j][it->k] >= 0);
			if (iter !=0)
				model.add(z[it->i][it->j][it->k] == 0);
		}
		// add tight edges contraint
		for(vector<pair<int,int> >::iterator it = tight_edges.begin(); it != tight_edges.end(); it++){
			IloExpr expr(env);
			expr += y[it->first][it->second];
			int temp[3];
			temp[0]= it->first;
			temp[1] = it->second;
			for (int k = 0; k < R.Candidates(); k++){
				temp[2]	= k;
				int i =0;
				while (i < 3)
					if((temp[i%3] < temp[(i+1)%3] && temp[(i+1)%3] < temp[(i+2)%3]) || 
							(temp[i%3] > temp[(i+1)%3] && temp[(i+1)%3] > temp[(i+2)%3])){
						expr += z[temp[i%3]][temp[(i+1)%3]][temp[(i+2)%3]];
						break;
					}else 
						i++;
			}
			model.add(expr <= 0);
		}
		stringstream st;
		st << iter++;
		string name("./drp_lp/drp_");
		name += st.str();
		name += ".lp";
		const char* n = name.c_str();	
		cplex.exportModel(n);
		//delete n;
		//n = NULL;
		//delete n;
		time_t start, end;
		time(&start);
		cplex.setOut(env.getNullStream());
		if ( !cplex.solve() ) {
			env.error() << "Failed to optimize LP." << endl;
			throw (-1);
		}
		time(&end);
		//create the solution
		s.time = difftime(end,start);
		s.obj_value = cplex.getObjValue();	
		// update te drp_solution struct
		for(map<cycle,int>::iterator it = Cycle.GetIterator(); it !=Cycle.EndIterator(); it++ ){
			drp_solution.z[it->second] = cplex.getValue(z[it->first.first.first][it->first.first.second][it->first.second]);
			//env.out() <<  cplex.getValue(z[it->first.first.first][it->first.first.second][it->first.second]) << endl; 
			//cout << "( " << it->first.first.first << " , " << it->first.first.second << ", " << it->first.second << " ) -> ( " << it->second << " )" <<  endl;	
		}
		for(map<pair<int,int>,int>::iterator it = Edge.GetIterator(); it !=Edge.EndIterator(); it++){
			drp_solution.y[it->second] = cplex.getValue(y[it->first.first][it->first.second]);
			//env.out() <<   cplex.getValue(y[it->first.first][it->first.second]) << endl;
			//cout << "( " << it->first.first << " , " << it->first.second << " ) -> ( " << it->second << " )" <<  endl;		
		}
		cout << "DRP LP OBJ Value is  = " << s.obj_value << endl;
		cout << "drp LP solution vector  y = { " << drp_solution.y << " }" << endl;
		cout << "drp LP solution vector z = { " << drp_solution.z << " }" << endl;
		
		for(int i = 0; i < drp_solution.y.size(); ++i ){
			pair<int,int> e = Edge.GetEdgeByLabel(i);
			cout << " y ["<< e.first<< "," << e.second << "]= " << drp_solution.y[i]<< endl;
		}
		for(int i = 0; i < drp_solution.z.size(); ++i ){
			pair<pair<int,int>,int > c = Cycle.GetCycleByLabel(i);
			cout << " z ["<< c.first.first<< "," << c.first.second <<"," << c.second << "]= " << drp_solution.z[i]<< endl;
		}

		//cout << "The LP solution is : " << s.sol_vector << endl;	
		env.end();
	}
	catch ( IloException & e){
		cerr << "Concert exception caught: " << e << endl;
	} catch (...){
		cerr << "Unknown exception caught" << endl;
	}

}
solution PrimalDualSolver::SolvePrimalDual(){
	solution s(R.Candidates());
	EdgeManager Edge;
	CycleManager Cycle;
	Edge.Initial(R.Candidates());
	Edge.Print();
	Cycle.Initial(R.Candidates());
	// Cycle.Print();
	vector<vector<dicycle> > ConstManager;
	int size_edge = R.Candidates()*(R.Candidates()-1);
	ConstManager.resize(size_edge);
	InitialConstraintManager(ConstManager, Edge, Cycle,size_edge);
	//PrintConstManager(ConstManager, Edge);
	dual_sol_vectors dual_solution(R.Candidates());
	dual_sol_vectors drp_solution(R.Candidates());
	vector<pair<int,int> > tight_edges;
	vector<dicycle> tight_cycles;
	try{
		if (!DualFeasibile(ConstManager, dual_solution,Edge,Cycle, tight_edges, tight_cycles)){
			cerr << "dual infesiblity in presolve" << endl;
			throw(-1);
		}
		int iteration = 0;
		double teta =0;
		while (true){	
			cout << "#####################################################"<< endl;
			cout << "iteration = " << iteration++ << endl;
			cout << "The dual soluiton value is = " << SolutionValue(dual_solution) << endl;
			//SolveDRP(tight_edges,tight_cycles,Edge,Cycle,drp_solution);
			CombinatorialDRPSolver(tight_edges,tight_cycles,Edge,Cycle,drp_solution);
			/*	
				cout << "#-----------------------------------------------------#"<< endl;
				cout << "  Variables values begin of iteration " << iteration << endl;
				cout << "dual solution vector  y = { " << dual_solution.y << " } "<<  endl;
				cout << "dual solution vector z = { "	<< dual_solution.z << "} " <<endl;
				cout << "drp solution vector  y = { " << drp_solution.y << " }" << endl;
				cout << "drp solution vector z = { " << drp_solution.z << " }" << endl;
				cout << "tight edges are : " << tight_edges << endl;
				cout << "tight cycles are : " << tight_cycles << endl;
				cout << "Teta = " << teta << endl;
				cout << "#--------------------------------------------------------#" << endl;
				*/	

			//check if the drp_solution value is 0
			if (SolutionValue(drp_solution)== 0){
				cout << "optimal dual is found, drp solution value is 0 " << endl;
				break;
			}
			teta = ComputeTeta(ConstManager,dual_solution,drp_solution, Edge, Cycle);			
			//cout << "teta is :" << teta << endl;
			if (teta <=0){
				cerr << "unacceptable teta = " << teta <<  endl;
				throw(-2);
			}
			//update the dual solution 
			for(int i =0; i < dual_solution.y.size(); i++)
				dual_solution.y[i] += teta*drp_solution.y[i];
			for(int i =0; i < dual_solution.z.size(); i++)
				dual_solution.z[i] += teta*drp_solution.z[i];
			cout << "dual solution "  << endl << " y = { " ;
			PrintVector(dual_solution.y); 
			cout << " } " << endl;;
			cout << " z = { "	<< dual_solution.z << "} " <<endl;
			cout << "drp solution " << endl << " y = { " ;
			PrintVector(drp_solution.y);
			cout  << " }" << endl;
			cout << " z = { " << drp_solution.z << " }" << endl;
			cout << "tight edges are : " << tight_edges << endl;
			cout << "tight cycles are : " << tight_cycles << endl;
			cout << "Teta = " << teta << endl;	
			cout << "#####################################################"<< endl;
			tight_edges.clear();
			tight_cycles.clear();
			if (!DualFeasibile(ConstManager, dual_solution,Edge,Cycle, tight_edges, tight_cycles)){
				cerr << "dual infesiblity" << endl;
				throw(-1);
			}
		}
		cout << "The dual optimal obj value is : "  << SolutionValue(dual_solution) << endl; ;
	}catch(int e){
		cout << "AN exception occured. Exception Nr." << e << endl;
	}

	//check the dual solution feasibility and compute the tight edges and cycles
	//while not drp zero solution do
	//	solve the drp
	//	check if the drp solutio is 0  
	//	compute the teta 
	//	compuet the dual solution
	//	clear the tight cycle and edge tight vectors
	//	check the dual feasibility
	//	 updated tight cycle and edges
	return s;	

}


solution PrimalDualSolver::SolveLPDual(){
	solution s(R.Candidates());
	try{
		IloEnv env;
		IloModel model(env);
		IloCplex cplex(model);
		NumVarMatrix y(env, R.Candidates());
		for(int i = 0; i < R.Candidates(); i++)
			y[i] = IloNumVarArray(env, R.Candidates(),-IloInfinity,IloInfinity,ILOFLOAT);
		for(int i =0; i < R.Candidates(); i++)
			for(int j =0; j < R.Candidates(); j++){
				stringstream s;
				s << i+1 << "," << j+1;
				string name("y_");
				name += s.str();
				const char* n = name.c_str();	
				y[i][j].setName(n);
			}
		ThreeDMatrix z(env,R.Candidates());
		for (int i = 0; i < R.Candidates(); i++){
			z[i] = NumVarMatrix(env,R.Candidates());
			for(int j =0; j < R.Candidates(); j++)
				//	z[i][j]= IloNumVarArray(env,R.Candidates(),-IloInfinity,0,ILOFLOAT);
				z[i][j]= IloNumVarArray(env,R.Candidates(),0,IloInfinity,ILOFLOAT);

		}
		for(int i =0; i < R.Candidates(); i++)
			for(int j =0; j < R.Candidates(); j++)
				for(int k =0; k < R.Candidates(); k++){
					stringstream s;
					s << i+1 << "," << j+1 << ","<<  k+1;
					string name("z_");
					name += s.str();
					const char* n = name.c_str();	
					z[i][j][k].setName(n);
				}
		//generate the obj_function
		IloExpr obj_fun(env);
		for(int i = 0; i < R.Candidates(); i++)
			for(int j = i+1; j < R.Candidates(); j++)
				obj_fun +=  y[i][j];
		for(int i =0; i < R.Candidates(); i++)
			for(int j = 0; j < R.Candidates(); j++)
				for(int k = 0; k < R.Candidates(); k++)
					if (i!=j && j!=k && k!=i)
						if (!( (j > i && j >k) || (j < i && j < k)) )
							//	obj_fun += 2*(z[i][j][k]);
							obj_fun += z[i][j][k];
		model.add(IloMaximize(env, obj_fun));
		//add simple constraints 
		//IloRangeArray c(env); 
		for(int i =0; i < R.Candidates(); i++)
			for(int j = i+1; j < R.Candidates(); j++)
				model.add(y[i][j] - y[j][i] == 0);

		/*	
			for (int i = 0; i < R.Candidates(); i++)
			for (int j = 0; j < R.Candidates(); j++)
			if (i != j){
			IloExpr exp(env);
			exp = y[i][j];
			for(int k = 0; k < R.Candidates(); k++)
			if( i !=k && j != k )
			exp += z[i][j][k];
			model.add(exp <= R.Get_m_Array()[j][i] );
			}		
			*/
		for(int i = 0; i < R.Candidates(); i++)
			for(int j =0 ; j < R.Candidates(); j++){
				//	model.add(z[i][i][i] == 0);
				//	model.add(z[i][i][j] == 0);	
				//	model.add(z[i][j][i] == 0);
				//	model.add(z[j][i][i] == 0);
			}
		for(int i = 0; i < R.Candidates(); i++)
			for(int j =0; j < R.Candidates(); j++)
				for(int k = 0; k < R.Candidates(); k++)
					if((j  >= i && j >= k) || ( j < i && j < k ))
						model.add(z[i][j][k] == 0);
		for (int i = 0; i < R.Candidates(); i++)
			for (int j = i +1; j < R.Candidates(); j++){
				IloExpr exp(env);
				IloExpr exp_r(env);
				exp = y[i][j];
				exp_r = y[j][i];
				for(int k = 0; k < R.Candidates(); k++)
					if( i !=k && j != k){
						if (k > j){
							exp += z[i][j][k];
							exp_r += z[k][j][i];
						}else if( i<k && k<j){
							exp += z[j][k][i];
							exp_r += z[i][k][j];
						}else{
							exp += z[k][i][j];
							exp_r += z[j][i][k];
						}
					}
				model.add(exp <= R.Get_m_Array()[j][i] );		
				model.add(exp_r <= R.Get_m_Array()[i][j] );			
			}
		for(int i = 0; i < R.Candidates(); i++)
			model.add(y[i][i] == 0);
		// add z_ijk =0	

		//time setting
		cplex.exportModel("dual.lp");
		time_t start, end;
		time(&start);
		cplex.setOut(env.getNullStream());
		if ( !cplex.solve() ) {
			env.error() << "Failed to optimize LP." << endl;
			throw (-1);
		}
		time(&end);
		//fill the solution
		s.time = difftime(end,start);
		s.obj_value = cplex.getObjValue();	

		/*for(int i =0; i < R.Candidates(); i++)
		  for(int j =0; j < R.Candidates(); j++)
		  s.sol_vector[i][j] = cplex.getValue(x[i][j]);*/
		//cout << "Time is = " << s.time << endl;
		cout << "Regular LP OBJ Value is  = " << s.obj_value << endl;
		//cout << "The LP solution is : " << s.sol_vector << endl;	
		IloNumArray duals(env);
		//cplex.getDuals(duals, c);
		//env.out() << duals;
		env.end();
	}
	catch ( IloException & e){
		cerr << "Concert exception caught: " << e << endl;
	} catch (...){
		cerr << "Unknown exception caught" << endl;
	}
	return s;
}





/////

double PrimalDualSolver::OrderingValue(vector<int> ordering){
	int size = ordering.size();
	double sum = 0;
	for(int i = 0 ; i < size; i++)
		for(int j = i+1; j < size ; j++)
			sum += R.Get_m_Array()[ordering[i]][ordering[j]];
	return sum;
}
IloNumArray PrimalDualSolver::OrderToNumArray(IloEnv env, vector<int> order){
	IloNumArray sVal(env);
	int size = order.size();
	for(int i = 0; i < size; i++)
		for(int j = 0; j < size; j++)
			if (i == j )
				sVal.add(0);
			else if ( i < j )
				sVal.add(1);
			else 
				sVal.add(0);
	return sVal;
}
vector<vector<int> > PrimalDualSolver::GetSubRanking(vector<int> partial_order){
	// get the subatrix related to ranking of candidates in the ve
	//vector of partial_order
	int size = partial_order.size();
	vector<vector<int> > rank;
	rank.resize(size);
	for(int i = 0; i < size; i++)
		rank[i].resize(size);
	for(int i = 0; i < size; i++)
		for(int j =0; j < size; j++)
			rank[i][j] =0;
	for(int i = 0; i < size; i++)
		for(int j = 0; j < size ; j++)
			rank[i][j] = R.Get_m_Array()[partial_order[i]][partial_order[j]];
	return rank;
}

