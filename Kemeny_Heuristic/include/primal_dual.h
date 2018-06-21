
/*
 * primal_dual.h
 *
 *  Created on: Feb 11, 2015
 *      Author: eia2
 */

#ifndef PRIMAL_DUAL_H_
#define PRIMAL_DUAL_H_


#include "ranking.h"
#include "Functions.h"
#include "pair_search_tree.h"
#include "adjlist.h"
typedef IloArray<IloBoolVarArray> BoolVarMatrix;
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix> ThreeDMatrix;

class PrimalDualSolver
{    
	Ranking  &R;
public :
	PrimalDualSolver(Ranking& r):R(r){}
	~PrimalDualSolver(){};
	solution SolveLPDual();
	solution  SolvePrimalDual();
	void SolveDRP(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycle, EdgeManager& Edge, CycleManager& Cycle,dual_sol_vectors& drp_solution);
	void CombinatorialDRPSolver(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycle, EdgeManager& Edge, CycleManager& Cycle,dual_sol_vectors& drp_solution);
	double OrderingValue(vector<int> ordering);
	double OrderingValue(vector<int>& order, double obj_val, int object, int old_pos, int new_pos);
	vector<vector<int> > GetSubRanking(vector<int> partial_order);
	IloNumArray OrderToNumArray(IloEnv env, vector<int> order);

	void InitialConstraintManager(vector<vector<dicycle> >& ConstManager, EdgeManager& Edge, CycleManager& Cycle, int size_edge);
	void PrintConstManager(vector<vector<dicycle> >& ConstManager, EdgeManager& Edge);
	double ComputeTeta(vector<vector<dicycle> >& ConstManager, dual_sol_vectors& dual_solution, dual_sol_vectors& drp_solution, 
		EdgeManager& Edge, CycleManager& Cycle);
	bool DualFeasibile(vector<vector<dicycle> >& ConstManager, dual_sol_vectors& dual_solution, 
		EdgeManager& Edge, CycleManager& Cycle, vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycles);
	double SolutionValue(dual_sol_vectors& v);
	void PrintVector( vector<double>& s );
	vector<AdjList> CreateGraph(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycles, 
		EdgeManager& Edge, CycleManager& Cycle, dual_sol_vectors& drp_solution, vector<int>& nTightEdgeInCycle, vector<int>& nReverseTightEdgeInCycle);
	solution Test_CombinatorialDRP_Random_Input_Graph();
	void Generate_Random_Tight_Edge_Cycle(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycles);
};
	//helprer functions
	bool BothInTightEdgeSet(vector<pair<int,int> >& tight_edges, pair<int,int> edge);
	bool BothInTightEdgeSet(vector<pair<int,int> >& tight_edges, int i , int j);
	bool InTightEdgeSet(vector<pair<int,int> >& tight_edges,int i,int j);	
	int FindMax(vector<int>& n);
	int FindMax2(vector<int>& n, vector<int>& r,vector<color>& vertex_status);
	bool AllVertexVisited(vector<color>& vertex_status);
	bool isTerminal(vector<int>& nTightEdgeInCycle, vector<int>& nReverseTightEdgeInCycle,int u);
	bool isValidMiddleNode(vector<int>& nTightEdgeInCycle,vector<int>& nReverseTightEdgeInCycle,int u);

#endif /* PRIMAL_DUAL_H_ */
