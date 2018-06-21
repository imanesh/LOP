/*
 *experiments.h
 *
 *
 *
 */


#include"ranking.h"
#include "new_ilp_solver.h"
#include "old_ilp_solver.h"
#include "old_lp_solver.h"
#include "triple_solver.h"
#include "all_triple_formul_solver.h"
#include "heuristic_solver.h"
#include "primal_dual.h"
#include "grasp.h"

void RunGrasp(const char* s, int gsp_cons_method, float alpha);
void RunIterativeMIP(const char* s);
int  mip_heuristic(const char* s );
void experiments(const char* s);
