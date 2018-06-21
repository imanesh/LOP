/*
 * master_header.h
 *
 *  Created on: Jan 14, 2014
 *      Author: eia2
 */

#ifndef MASTER_HEADER_H_
#define MASTER_HEADER_H_



#include <ilcplex/ilocplex.h>
#include <iostream>
#include <fstream>
#include <ostream>
#include <cmath>
#include <vector>
#include <set>
#include <assert.h>
#include <map>
#include <time.h>
#include <sstream>
#include <queue>
#include <algorithm>
#include <list>
#include "CycleManager.h"
#include "EdgeManager.h"
//#include <boost/bimap.hpp>
//#include <unordered_set>
#include "tclap/CmdLine.h"
#include <Eigen/Dense>
#include <stack>
enum color{white, gray, black};
enum solve_status {fully, restricted};
using namespace std;
ILOSTLBEGIN



struct dicycle{
	int i , j, k;
	dicycle(int s=0,int t=0, int w=0):i(s), j(t), k(w){}
};



struct kerlin_struct{
	pair<int, int> p;
	int gain;
	kerlin_struct(pair<int,int> a, int g):gain(g){
		if (a.first < a.second){
			p.first = a.first ;
			p.second = a.second;
		}
		else if (a.first > a.second)
			p.first = a.second;
			p.second = a.first;
	}
	bool operator==(kerlin_struct a) const{
		    if((p.first==a.p.first && p.second == a.p.second) ||
				(p.second == a.p.first && p.first == a.p.second)) return true;
			    else return false;
	}
	bool operator <(kerlin_struct a) const{
		if ((p.first < a.p.first) || 
				((p.first == a.p.first) &&(p.second < a.p.second)))
			return true;
			else return false;	
	}
};




struct TS{
	int a;
	vector<int> v;
	bool operator<(const TS& rhs) const{
		return a < rhs.a;
	}
	TS(int val, vector<int> vec):a(val), v(vec){};
};


struct  Solution
{
	double best_solution_value;
	double current_solution_value;
	vector<vector<int> > best_solution;
	vector<vector<int> > current_solution;
	Solution(int n){
		best_solution.resize(n);
		for(int i = 0; i < n; i++)
			best_solution[i].resize(n);
		current_solution.resize(n);
		for(int i = 0; i < n; i++)
			current_solution[i].resize(n);
	}
};


struct  solution{ 
	//const char* mod;
	double obj_value;
	double time;
	vector<vector<double> > sol_vector;
	solution(int n):/*mod("lp"),*/time(0),obj_value(0){
		sol_vector.resize(n);
		for(int i = 0; i < n; i++)
			sol_vector[i].resize(n);
	}
};

struct dual_sol_vectors{
	vector<double> y;
	vector<double> z;
	dual_sol_vectors(int n){
		y.resize(n*(n-1));
		for(int i = 0; i < n*(n-1);i++)
			y[i] =0;
		z.resize((n*(n-1)*(n-2))/3);
		for(int i =0; i < (n*(n-1)*(n-2))/3;i++)
			z[i] = 0;
	}
};


/**
 * Wrap any debug-only statements using DEBUG_ONLY(...)
 */
#ifdef NDEBUG
#define DEBUG_ONLY(statement)
#else
#define DEBUG_ONLY(statement) statement
#endif



#endif /* MASTER_HEADER_H_ */
