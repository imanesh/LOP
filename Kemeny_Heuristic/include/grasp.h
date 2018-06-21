/*
 * grasp.h
 *
 *  Created on: May 16, 2016
 *      Author: Ehsan
 */

#ifndef GRASP_H_
#define GRASP_H_


#include "ranking.h"
#include "Functions.h"
#include "pair_search_tree.h"
typedef IloArray<IloBoolVarArray> BoolVarMatrix;
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix> ThreeDMatrix;

class Grasp
{    
	Ranking  &R;
	float alpha;
public :
	Grasp(Ranking& r):R(r){alpha = 0.7;}
	~Grasp(){};

	solution SolveILP();
	solution SolveILP(vector<vector<double> >& matrix);
	solution SolveLP();
	solution SolveLPDual();
	solution SolveLP(vector<vector<double> >& matrix);	
	
	double OrderingValue(vector<int> ordering);
	double OrderingValue(vector<int>& order, double obj_val, int object, int old_pos, int new_pos);
	vector<vector<int> > GetSubRanking(vector<int> partial_order);
	IloNumArray OrderToNumArray(IloEnv env, vector<int> order);
	double SolutionValue(dual_sol_vectors& v);
	void PrintVector( vector<double>& s );


	void local_search_improvement(vector<int>& initial_solution);
	void UpdateVariableHistory(vector<vector<int> >& history, vector<int>& order);
	vector<int> GenerateRandInitialSolution();
	vector<int> ConstructionMethodOne();	
	vector<int> ConstructionMethodTwo();
	vector<int> ConstructionMethodThree();
	vector<vector<int> > grasp(vector<int>& initial_solution,int gsp_cons_method, float alpha, int MaxItr);
	vector<vector<int> > grasp(int gsp_cons_method, float alpha, int MaxItr);
	void TestGrasp();
	double insert(vector<int>& order, int pos, int m);
};

#endif /* GRASP_H_ */
