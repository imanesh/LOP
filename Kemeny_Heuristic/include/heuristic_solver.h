/*
 * heuristic_solver.h
 *
 *  Created on: March 28, 2014
 *      Author: eia2
 */

#ifndef HEURISTIC_SOLVER_H_
#define HEURISTIC_SOLVER_H_


#include "ranking.h"
#include "Functions.h"
#include "pair_search_tree.h"
#include "grasp.h"
#include "Graph.h"
typedef IloArray<IloBoolVarArray> BoolVarMatrix;
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix> ThreeDMatrix;

class HeurSolver
{    
	Ranking  &R;
public :
	HeurSolver(Ranking& r):R(r){}
	~HeurSolver(){};

	solution SolveILP();
	solution SolveILP(vector<vector<double> >& matrix);
	solution SolveLP();
	solution SolveLPDual();
	solution SolveLP(vector<vector<double> >& matrix);	
	solution SolveLPIterat();
	solution  SolvePrimalDual();
	void SolveDRP(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycle, EdgeManager& Edge, CycleManager& Cycle,dual_sol_vectors& drp_solution);
	void Combinatorial_DRP_Solve(vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycle, EdgeManager& Edge, CycleManager& Cycle,dual_sol_vectors& drp_solution);
	solution LPRoundingHeuristic(solution& lp);	
	void MIP_HeuristicTest();
	void MIP_Heuristic();
	int MIP_Heuristic(int grasp_cons_id);
	void MIP_Heuristic_copy();
	void MIP_Heuristic_150();
	void MIP_Heuristic_200();
	void MIP_Heuristic_500();	
	vector<int> Preprocessing(int grasp_cons_id);
	vector<int> Preprocessing();
	void Test();
	vector<int> IterativeRelaxing();
	solution CreateInitialSolution(solution& lp);
	double OrderingValue(vector<int> ordering);
	double OrderingValue(vector<int>& order, double obj_val, int object, int old_pos, int new_pos);
	vector<vector<int> > GetSubRanking(vector<int> partial_order);
	void CreateModel(IloEnv env,IloModel model, BoolVarMatrix x);
	void CreateModel(IloEnv env,IloModel model, NumVarMatrix x);
	vector<int> SolveSubProblem(vector<vector<int> > matrix, vector<int> cand_subset, IloNumArray st);
	vector<int> ImproveByCplex(vector<int> cand_subset, IloNumArray start_val);
	vector<int> LocalSearch(vector<int> cand_subset, IloNumArray start_val);
	vector<int> LocalSearch(vector<vector<int> > matrix, vector<int> cand_subset, IloNumArray start_val,  solve_status status, int p);
	vector<int> LastStepMIP(vector<int>& order);
	IloNumArray OrderToNumArray(IloEnv env, vector<int> order);
	void LPHeuristic();
	solution FixnSolve(vector<vector<double> >& diff_score_matrix, vector<int>& current_solution, int size, int threshold);
	void LPHeuristic(solution& lp, vector<int>& init_ordering);
	int ExistViolateDicycle(IloModel model,IloCplex cplex , NumVarMatrix x);
	void UpdateVarHistory(vector<vector<int> >& history, vector<int>& order);
	vector<int> LinKernighanLocalSearch(vector<int>& order, int size);
	void KernelSearch();
	vector<int> IterativeFixing(solution& lp, vector<int>& init_order, double threshold, int iteration);

	void InitialConstraintManager(vector<vector<dicycle> >& ConstManager, EdgeManager& Edge, CycleManager& Cycle, int size_edge);
	void PrintConstManager(vector<vector<dicycle> >& ConstManager, EdgeManager& Edge);
	double ComputeTeta(vector<vector<dicycle> >& ConstManager, dual_sol_vectors& dual_solution, dual_sol_vectors& drp_solution, 
		EdgeManager& Edge, CycleManager& Cycle);
	bool DualFeasibile(vector<vector<dicycle> >& ConstManager, dual_sol_vectors& dual_solution, 
		EdgeManager& Edge, CycleManager& Cycle, vector<pair<int,int> >& tight_edges, vector<dicycle>& tight_cycles);
	double SolutionValue(dual_sol_vectors& v);
	void PrintVector( vector<double>& s );

};

#endif /* HEURISTIC_SOLVER_H_ */
