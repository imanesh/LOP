/*
 * heuristic_solver.cpp
 *  Created on: Jan 14, 2014
 *  Author: eia2
 */
#include "grasp.h"
#define RC_EPS 1.0e-10


//helper function
double Grasp::insert(vector<int>& order, int pos, int m){
	double gain;
	//implement the insert move, inserting element in order[pos] in m-th position
	if (pos > m )
		for (int k =m; k<=pos-1 ; ++k)
			gain += (R.Get_m_Array()[order[pos]][order[k]]- R.Get_m_Array()[order[k]][order[pos]]);
	else
		for (int k =pos+1; k<= m ; ++k)
			gain += (R.Get_m_Array()[order[k]][order[pos]]- R.Get_m_Array()[order[pos]][order[k]]);
	return OrderingValue(order) + gain;
}


vector<int> update_insert_move(vector<int>& order, int pos, int m){
	vector<int> update_ordering = order;
	//iplement the insert move and update the ordering 
	int element = order[pos];
	if(pos > m){
		//cout << "pos is = " << pos << " and m is =  " << m << endl;
		assert(pos != 0);
		for(int i= pos-1; i >= m; --i)
			update_ordering[i+1] = update_ordering[i];
		update_ordering[m]= element;	
	}		
	else{
		for(int i= pos; i < m; ++i)
			update_ordering[i] = update_ordering[i+1];
		update_ordering[m]= element;	
	}
	return update_ordering;
}


void  Grasp::local_search_improvement(vector<int>& initial_solution)
{
	vector<int> best_solution;
	try{
		double best_obj_val =0;
		double obj_value = OrderingValue(initial_solution);
		//improvement phase
		for(int pos = 0; pos < R.Candidates(); ++pos){
			int gain = 0;        
			int gain_idx =0;    
			for(int m = 0; m < R.Candidates(); ++m)
				if (pos != m)
					if (gain < insert(initial_solution,pos,m)){	
						gain = insert(initial_solution,pos,m);
						gain_idx = m;
					}
			vector<int> temp_solution = update_insert_move(initial_solution,pos,gain_idx);
			if (obj_value < OrderingValue(temp_solution)){
				initial_solution = temp_solution;	
				obj_value = OrderingValue(initial_solution);
			}
		}
		//update the variable history matrix
		int value_current_sol =  OrderingValue(initial_solution);
		if (best_obj_val < value_current_sol){
			best_obj_val  = value_current_sol;
			best_solution = initial_solution;
		//	cout << "the best solution in grasp having obj valu = " << best_obj_val << endl;
		}
	}
	catch (exception& e){
		cerr << "there is an exception in lcoal search improvement!!" << endl;
	}
	initial_solution = best_solution;
}




void Grasp::TestGrasp(){
	//vector<int> initial_solution = GenerateRandInitialSolution();	
	/*int inputs[] = {10, 0, 8, 1, 4, 6, 7, 5, 9, 2, 3 };
	  vector<int> initial_solution(inputs, inputs + sizeof(inputs) / sizeof(int) );
	  double obj_value = OrderingValue(initial_solution);
	  cout << "initial_solution = " << initial_solution << endl;
	  cout << "obj value " << obj_value << endl;
	  int pos = 5;
	  int m = 8;

	  int gain = insert(initial_solution, pos, m);	
	  vector<int> temp_solution = update_insert_move(initial_solution,pos,m);			
	  cout << "update solution is = " << temp_solution << endl;
	  cout << "gain is = " << gain << endl;
	  cout << "update obj valu is = " << OrderingValue(temp_solution) << endl; 
	  */
	//for (int i = 0; i< 10; ++i)
	ConstructionMethodOne();
}







vector<vector<int> > Grasp::grasp(vector<int>& initial_solution,int gsp_cons_method, float alpha,int MaxItr)
{
	this->alpha = alpha;
	vector<vector<int> > history;
	try{
		history.resize(R.Candidates());
		for(int i =0 ; i < R.Candidates(); ++i)
			history[i].resize(R.Candidates());
		//initiate history
		for(int i =0; i < R.Candidates(); ++i)
			for(int j = 0; j < R.Candidates() ;++j)
				history[i][j] = 0;
		double best_obj_val =0;
		vector<int> best_solution;
		int i = 0; 

		//grasp main loop
		while(i < MaxItr){ 
			//construction phase
			//vector<int> initial_solution = GenerateRandInitialSolution();
			switch (gsp_cons_method) {
				case 1 :  initial_solution = ConstructionMethodOne();
						  break;
				case 2 :  initial_solution = ConstructionMethodTwo();
						  break;
				case 3 :  initial_solution = ConstructionMethodThree();
						  break;
				default: cout << "non valid cons method" << endl; 
						break;
			}


			double obj_value = OrderingValue(initial_solution);
			//improvement phase
			for(int pos = 0; pos < R.Candidates(); ++pos){
				int gain = 0;         //   TBC 
				int gain_idx =0;       //  TBC
				for(int m = 0; m < R.Candidates(); ++m)
					if (pos != m)
						if (gain < insert(initial_solution,pos,m)){	
							gain = insert(initial_solution,pos,m);
							gain_idx = m;
						}

				//to test the insert
				//int tt = insert(initial_solution, pos, gain_idx);
				vector<int> temp_solution = update_insert_move(initial_solution,pos,gain_idx);		
				if (obj_value < OrderingValue(temp_solution)){
					initial_solution = temp_solution;	
					obj_value = OrderingValue(initial_solution);
		//			cout << "updates sol is = " << initial_solution << endl;
		//			cout << "sol updated and best value do far is = " << OrderingValue(temp_solution) << endl;
				}
			}
			//update the variable history matrix
			int value_current_sol =  OrderingValue(initial_solution);
			if (best_obj_val < value_current_sol){
				best_obj_val  = value_current_sol;
				best_solution = initial_solution;
			}


			UpdateVariableHistory(history, initial_solution);
			++i;		
		}
		//cout << "history is = "<< endl << history << endl;
		initial_solution = best_solution;
//		cout << "the best solution in grasp having obj valu = " << best_obj_val << endl;
	}
	catch (exception& e){
		cerr << "there is an exception in grasp!!" << endl;
	}
	return history;
}


vector<vector<int> > Grasp::grasp(int gsp_cons_method, float alpha, int MaxItr)
{
//	cout << "core grasp starting .. " << endl;
	vector<vector<int> > history;
	this->alpha = alpha;
	try{
		history.resize(R.Candidates());
		for(int i =0 ; i < R.Candidates(); ++i)
			history[i].resize(R.Candidates());
		//initiate history
		for(int i =0; i < R.Candidates(); ++i)
			for(int j = 0; j < R.Candidates() ;++j)
				history[i][j] = 0;
		double best_obj_val =0;
		vector<int> best_solution;
		int i = 0; 
		//grasp main loop
		while(i < MaxItr){
			 vector<int> initial_solution; 
			//construction phase
			//vector<int> initial_solution = GenerateRandInitialSolution();
			switch (gsp_cons_method) {
				case 1 :  initial_solution = ConstructionMethodOne();
						  break;
				case 2 :  initial_solution = ConstructionMethodTwo();
						  break;
				case 3 :  initial_solution = ConstructionMethodThree();
						  break;
				default: cout << "non valid cons method" << endl; 
						break;
			}
			double obj_value = OrderingValue(initial_solution);
			//improvement phase
			for(int pos = 0; pos < R.Candidates(); ++pos){
				int gain = 0;         //   TBC 
				int gain_idx =0;       //  TBC
				for(int m = 0; m < R.Candidates(); ++m)
					if (pos != m)
						if (gain < insert(initial_solution,pos,m)){	
							gain = insert(initial_solution,pos,m);
							gain_idx = m;
						}
				vector<int> temp_solution = update_insert_move(initial_solution,pos,gain_idx);
				if (obj_value < OrderingValue(temp_solution)){
					initial_solution = temp_solution;	
					obj_value = OrderingValue(initial_solution);
					//cout << "updates sol is = " << initial_solution << endl;
					//cout << "sol updated and best value do far is = " << OrderingValue(temp_solution) << endl;
				}
			}
			//update the variable history matrix
			int value_current_sol =  OrderingValue(initial_solution);
			if (best_obj_val < value_current_sol){
				best_obj_val  = value_current_sol;
				best_solution = initial_solution;
			}
			UpdateVariableHistory(history, initial_solution);
			++i;		
		}
		//cout << "the best solution in grasp having obj valu = " << best_obj_val << endl;
	}
	catch (exception& e){
		cerr << "there is an exception in grasp!!" << endl;
	}
	return history;
}


vector<int> Grasp::GenerateRandInitialSolution()
{
	vector<int> init_ordering(R.Candidates());
	for(int i = 0; i < R.Candidates(); i++)
		init_ordering[i] = i;
	random_shuffle(init_ordering.begin(), init_ordering.end());
	return init_ordering;
}

vector<int> Grasp::ConstructionMethodOne(){
	//cout << "construction method one started ...." << endl;
	double treshold;
	vector<int> e_1(R.Candidates(),-1);
	vector<int> ordering;
	list<int> U(R.Candidates(),0);
	int k =0;
	list<int>::iterator it;
	for(it = U.begin(); it!=U.end(); ++it)
		*it = k++;
	for(int iter = 0; iter<R.Candidates(); ++iter){
		vector<int> RCL;
		int max_e_1 = -IloInfinity;
		//compute the ei= \sum_c_ij
		list<int>::iterator it_j;
		for(it = U.begin(); it!=U.end(); ++it){
			int sum=0;	
			for(it_j = U.begin(); it_j!=U.end(); ++it_j)
				if(it != it_j)
					sum += R.Get_m_Array()[*it][*it_j];
			e_1[*it] = sum;
		}
		//find the maximum of  e_1
		vector<int>::iterator v;
		for(v = e_1.begin() ; v != e_1.end(); ++v)
			if (*v > max_e_1)
				max_e_1 = *v;

		treshold = alpha * max_e_1;
		for(int i  = 0; i < R.Candidates(); ++i)
			if (e_1[i] >= treshold)
				RCL.push_back(i);
		//select one elemnt randomley from RCL and add it to ordering
		//update the e_1 and U and RCL	
		int rcl_size = RCL.size();
		int index = 0;
		index = rand() % rcl_size;
		assert(index < rcl_size);
		ordering.push_back(RCL[index]);
		//remove index from U
		U.remove(RCL[index]);	
		e_1[RCL[index]] = -1;
	}
	assert(ordering.size() == R.Candidates());
//	cout << "construction method one finished ! "<< endl;
	return ordering;

}

vector<int> Grasp::ConstructionMethodTwo(){
//	DEBUG_ONLY(cout << "construction method two started ...." << endl);
	double treshold;
	int colmax =0;
	int colmax_idx;
	vector<int> e_2(R.Candidates(),-1);
	vector<int> ordering;
	list<int> U(R.Candidates(),0);
	int k =0;
	list<int>::iterator it;
	for(it = U.begin(); it!=U.end(); ++it)
		*it = k++;
	
	//find the colmax
	for(int i =0; i< R.Candidates(); ++i){
		int sum =0;
		for(int j  =0; j < R.Candidates(); ++j)
			sum+=R.Get_m_Array()[j][i];
		if (sum > colmax){
			colmax = sum;
			colmax_idx = i;
		}
	}
	for(int iter = 0; iter<R.Candidates(); ++iter){
		vector<int> RCL;
		int max_e_2 = -IloInfinity;
		//compute the ei= \sum_c_ij
		list<int>::iterator it_j;
		for(it = U.begin(); it!=U.end(); ++it){
			int sum=0;	
			for(it_j = U.begin(); it_j!=U.end(); ++it_j)
				if(it != it_j)
					sum += R.Get_m_Array()[*it_j][*it];
			e_2[*it] = colmax-sum;
		}
		//find the maximum of  e_2
		vector<int>::iterator v;
		for(v = e_2.begin() ; v != e_2.end(); ++v)
			if (*v > max_e_2)
				max_e_2 = *v;

		treshold = alpha * max_e_2;
		for(int i  = 0; i < R.Candidates(); ++i)
			if (e_2[i] >= treshold)
				RCL.push_back(i);
		//select one elemnt randomley from RCL and add it to ordering
		//update the e_1 and U and RCL	
		int rcl_size = RCL.size();
		int index = 0;
		index = rand() % rcl_size;
		assert(index < rcl_size);
		ordering.push_back(RCL[index]);
		//remove index from U
		U.remove(RCL[index]);	
		e_2[RCL[index]] = -1;
	}
	assert(ordering.size() == R.Candidates());
//	DEBUG_ONLY(cout << "construction method two finished ! "<< endl);
	return ordering;
}

vector<int> Grasp::ConstructionMethodThree(){
//	cout << "construction method three started ...." << endl;
	double treshold;
	vector<double> e_3(R.Candidates(),-1);
	vector<int> ordering;
	list<int> U(R.Candidates(),0);
	int k =0;
	list<int>::iterator it;
	for(it = U.begin(); it!=U.end(); ++it)
		*it = k++;
	for(int iter = 0; iter<R.Candidates(); ++iter){
		vector<int> RCL;
		double max_e_3 = -IloInfinity;
		//compute the ei= \sum_c_ij
		list<int>::iterator it_j;
		for(it = U.begin(); it!=U.end(); ++it){
			int sum_row=0;
			int sum_col =0;	
			for(it_j = U.begin(); it_j!=U.end(); ++it_j)
				if(it != it_j){
					sum_row += R.Get_m_Array()[*it][*it_j];
					sum_col += R.Get_m_Array()[*it_j][*it];
				}
			if(sum_col != 0)
				e_3[*it] = sum_row/sum_col;
			else 
				e_3[*it] = 0;
		}
		//find the maximum of  e_1
		vector<double>::iterator v;
		for(v = e_3.begin() ; v != e_3.end(); ++v)
			if (*v > max_e_3)
				max_e_3 = *v;

		treshold = alpha * max_e_3;
		for(int i  = 0; i < R.Candidates(); ++i)
			if (e_3[i] >= treshold)
				RCL.push_back(i);
		//select one elemnt randomley from RCL and add it to ordering
		//update the e_1 and U and RCL	
		int rcl_size = RCL.size();
		int index = 0;
		index = rand() % rcl_size;
		assert(index < rcl_size);
		ordering.push_back(RCL[index]);
		//remove index from U
		U.remove(RCL[index]);	
		e_3[RCL[index]] = -1;
	}
	assert(ordering.size() == R.Candidates());
//	cout << "construction method three finished ! "<< endl;
	return ordering;
}

void Grasp::UpdateVariableHistory(vector<vector<int> >& history, vector<int>& order){
	for(int i = 0 ; i < R.Candidates(); i++)
		for(int j = i+1; j < R.Candidates() ; j++)
			++history[order[i]][order[j]];

}



double Grasp::OrderingValue(vector<int> ordering){
	int size = ordering.size();
	double sum = 0;
	for(int i = 0 ; i < size; i++){	
		//	cout << "sum for candidates = " << ordering[i] << "  is = ";
		//	int s = 0;
		for(int j = i+1; j < size ; j++){
			//		s +=  R.Get_m_Array()[ordering[i]][ordering[j]];
			sum += R.Get_m_Array()[ordering[i]][ordering[j]];
		}
		//	cout << s << endl;
	}
	return sum;
}

IloNumArray Grasp::OrderToNumArray(IloEnv env, vector<int> order){
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
vector<vector<int> > Grasp::GetSubRanking(vector<int> partial_order){
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

void Grasp::PrintVector(vector<double>& s)
{
	int counter = 0;
	vector<double>::const_iterator i;
	for(i = s.begin(); i != s.end(); i = i+2){
		cout << *i << "  ";  // increase by 1 in order to match the convention indexing scheme
	}
}

double Grasp::SolutionValue(dual_sol_vectors& v){
	double obj_val_drp = 0;
	for(int i =0; i < v.y.size();i = i+2)
		obj_val_drp += v.y[i];
	for(int i= 0; i < v.z.size(); i++)
		obj_val_drp += v.z[i];
	return obj_val_drp;
}


