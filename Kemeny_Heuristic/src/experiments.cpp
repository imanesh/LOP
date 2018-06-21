/*
 * experiments.cpp
 *
 *
 *
 */

#include "experiments.h"

void experiments(const char* s){
/*
	string output_file_name;
	output_file_name = "experiments_results/";
	output_file_name += s;
	output_file_name += "_hello1.txt";
	//output_file_name = "experiments_results/Nt1d100.01_hello.txt";
	const char* b = output_file_name.c_str();
	ofstream out(b);
	out << "hello" << endl;
	cout << output_file_name << endl;
	out.close();
*/
	DEBUG_ONLY(std::cout << "Log file name" << std::endl);
	cout << "start the experiment" << endl;
	
	srand( time( NULL ) );
	Ranking kem_rank(s); 
	kem_rank.Initiate_m_Matrix_From_File();
	HeurSolver heur(kem_rank);
/*	cout << "experimt with LProung as an intial solution" << endl;
	for (int i=0; i < 10 ; ++i){
		int b =heur.MIP_Heuristic(0);
		cout  << b << endl;
	}	
*/	
	cout << "experimt with grasp 1 as an intial solution" << endl;
	for (int i=0; i < 5 ; ++i){
		int b =heur.MIP_Heuristic(1);
		cout  << b << endl;
	}	
	cout << "experimt with grasp 2 as an intial solution" << endl;
	for (int i=0; i < 5 ; ++i){
		int b =heur.MIP_Heuristic(2);
		cout  << b << endl;
	}	
	cout << "experimt with grasp 3 as an intial solution" << endl;
	for (int i=0; i < 5 ; ++i){
		int b =heur.MIP_Heuristic(3);
		cout  << b << endl;
	}
		
}




void RunGrasp(const char* s, int gsp_cons_method, float alpha){
	cout << "start running grasp......" << endl;
	
	srand( time( NULL ) );
	Ranking kem_rank(s); 
	kem_rank.Initiate_m_Matrix_From_File();

	Grasp g(kem_rank);
	int iteration = 100;
	g.grasp(gsp_cons_method,alpha,iteration);
//	g.TestGrasp();
//	OldILPSolver ilp(kem_rank);
//	ilp.SolveILP();

	cout << "This is the end " << endl;

}


void RunIterativeMIP(const char* s){
	cout << "iteartive rouding mip heuristic is starting ..." << endl;
	srand( time( NULL ) );
	Ranking kem_rank(s); 
	kem_rank.Initiate_m_Matrix_From_File();
	HeurSolver heur(kem_rank);
	heur.IterativeRelaxing();
	OldILPSolver ilp(kem_rank);
	//ilp.SolveILP();
	cout << "This is the end " << endl;

}



int  mip_heuristic(const char* s)
{
	DEBUG_ONLY(std::cout << "Log file name" << std::endl);
	cout << "start the experiment" << endl;
	
	srand( time( NULL ) );
	//Ranking kem_rank;
	Ranking kem_rank(s); 
	kem_rank.Initiate_m_Matrix_From_File();

	HeurSolver heur(kem_rank);
	//old_lp.SolveLP();
//	old_lp.CreateInitialSolution();
	//cout << "The LP Solution is : "<<  old_lp.GetValLP()<< endl;
//	heur.Test();
	//heur.MIP_Heuristic();
	int b =heur.MIP_Heuristic(3);
	cout << "b" << b << endl;
	//	heur.KernelSearch();
//	heur.MIP_Heuristic_150();
//	heur.MIP_Heuristic_200();
//	heur.MIP_Heuristic_500();
//	heur.SolveLPDual();
	//heur.SolveLP();
//	heur.SolvePrimalDual();
//	heur.SolveLPIterat();
//	heur.SolveILP();
//	heur.LPHeuristic();
   //old_lp.LPRoundingHeuristic();
   //old_lp.IterativeSequenceFixingHeuristic(4);
   //old_lp.IterativeLPVariableFixingHeuristic_2(4);  
	//OldILPSolver ilp(kem_rank);
	//ilp.SolveILP();

	cout << "*************************************************************"<< endl;
	//cout << "The LP Solution is : "<<  old_lp.GetValLP()<< endl;
	//cout << "The running time of the Iterated LP is : " << old_lp.GetRTLP() <<" milisecond" << endl;
	//cout << "The number of constraints in LP is : " << old_lp.GetnConstraintLP() << endl<<endl;
//	cout << "The  IP Solution is : "<<  ilp.GetValIP()<< endl;
    //ilp.PrintResults();
	cout << "*************************************************************"<< endl;
	cout << "This is the end " << endl;
	return 0;

}


