SYSTEM     = x86-64_osx
LIBFORMAT  = static_pic
#------------------------------------------------------------
# When you adapt this makefile to compile your CPLEX programs
# please copy this makefile and set CPLEXDIR and CONCERTDIR to
# the directories where CPLEX and CONCERT are installed.
#------------------------------------------------------------
CPLEXDIR      = /Users/Ehsan/Applications/IBM/ILOG/CPLEX_Studio1261/cplex
CONCERTDIR    = /Users/Ehsan/Applications/IBM/ILOG/CPLEX_Studio1261/concert
CURRENTDIR    = /Users/Ehsan/Documents/workspace_c++/Kemeny_Heuristic
INCLUDEDIR    = $(CURRENTDIR)/include
SRCDIR        = $(CURRENTDIR)/src
OBJDIR        = $(CURRENTDIR)/obj
BOOSTINCDIR   =  /opt/local/include
BOOSTLIBDIR   = /opt/local/lib
# ---------------------------------------------------------------------
# Compiler selection 
# ---------------------------------------------------------------------
#CCC = g++
CCC = clang++ -O0
CC  = clang -O0
JAVAC = javac
# ---------------------------------------------------------------------
# Compiler options 
# ---------------------------------------------------------------------
#CCOPT = -m64 -O0 -fPIC -fexceptions -DNDEBUG -DIL_STD -g
CCOPT = -m64 -O -fPIC -fexceptions  -DIL_STD -stdlib=libstdc++
COPT  = -m64 -fPIC
JOPT  = -classpath $(CPLEXDIR)/lib/cplex.jar -O
 # ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------
CPLEXBINDIR   = $(CPLEXDIR)/bin/$(BINDIST)
CPLEXJARDIR   = $(CPLEXDIR)/lib/cplex.jar
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
#CCLNFLAGS = -L$(CPLEXLIBDIR) -lilocplex -lcplex -L$(CONCERTLIBDIR) -lconcert -lm -pthread -L$(CURRENTDIR)
CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR) -L$(BOOSTLIBDIR)
CLNDIRS   = -L$(CPLEXLIBDIR)
CCLNFLAGS = -L$(CONCERTLIBDIR) -lconcert  -L$(CPLEXLIBDIR)  -lilocplex -lcplex -m64 -lm -lpthread -framework CoreFoundation -framework IOKit  -stdlib=libstdc++ -L$(CURRENTDIR) -L$(BOOSTLIBDIR) 
CLNFLAGS  = -lcplex -m64 -lm -lpthread -framework CoreFoundation -framework IOKit
JAVA      = java  -d64 -Djava.library.path=$(CPLEXDIR)/bin/x86-64_osx -classpath $(CPLEXJARDIR):
# --------------------------------------------------------------------
#  Debug option
# __________________________________________________________________
#DEBUG ?= 1
#ifeq (DEBUG, 1)
#	CCOPT += -DDEBUG
#else
#	CCOPT += -DNDEBUG
#endif
# -------------------------------------------------------------------

all:
	make all_cpp
debug: CCOPT += -DDEBUG -g
debug: COPT += -DDEBUG -g
debug: ./kemeny

release: CCOPT += -DNDEBUG 
release: COPT += -DNDEBUG 
release: ./kemeny



execute: all
	make execute_cpp
CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include
EXDIR         = $(CPLEXDIR)/examples
EXINC         = $(EXDIR)/include
EXDATA        = $(EXDIR)/data
EXSRCCPP      = $(EXDIR)/src/cpp
CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) -I$(CURRENTDIR) -I$(BOOSTINCDIR) -I$(INCLUDEDIR) 
#------------------------------------------------------------
#  make all      : to compile the examples. 
#  make execute  : to compile and execute the examples.
#------------------------------------------------------------
CPP_EX = kemeny
all_cpp: $(CPP_EX)
execute_cpp: $(CPP_EX)
	 ./kemeny
OBJECTS = $(OBJDIR)/main.o $(OBJDIR)/Functions.o $(OBJDIR)/ranking.o $(OBJDIR)/experiments.o \
	$(OBJDIR)/triple_solver.o $(OBJDIR)/all_triple_formul_solver.o \
	 $(OBJDIR)/new_ilp_solver.o $(OBJDIR)/old_ilp_solver.o $(OBJDIR)/map_data_structure.o \
	$(OBJDIR)/old_lp_solver.o $(OBJDIR)/heuristic_solver.o $(OBJDIR)/pair_search_tree.o  $(OBJDIR)/grasp.o \
	 $(OBJDIR)/EdgeManager.o $(OBJDIR)/CycleManager.o $(OBJDIR)/primal_dual.o $(OBJDIR)/adjlist.o $(OBJDIR)/Graph.o
# ------------------------------------------------------------
clean :
	/bin/rm -rf *.o *~ *.class $(OBJDIR)/*.o
	/bin/rm -rf $(C_EX) $(CPP_EX) $(CURRENTDIR)/bin/* 
	/bin/rm -rf *.mps *.ord *.sos *.lp *.sav *.net *.msg *.log *.clp
# ------------------------------------------------------------
kemeny : $(OBJECTS)
	$(CCC) $(CCFLAGS) $(OBJECTS) -o $(CURRENTDIR)/bin/kemeny $(CCLNFLAGS)
$(OBJDIR)/main.o : $(SRCDIR)/main.cpp $(INCLUDEDIR)/ranking.h $(INCLUDEDIR)/new_ilp_solver.h $(INCLUDEDIR)/primal_dual.h $(INCLUDEDIR)/heuristic_solver.h $(INCLUDEDIR)/old_ilp_solver.h $(INCLUDEDIR)/experiments.h $(INCLUDEDIR)/old_lp_solver.h $(INCLUDEDIR)/triple_solver.h $(INCLUDEDIR)/all_triple_formul_solver.h $(INCLUDEDIR)/EdgeManager.h $(INCLUDEDIR)/CycleManager.h  $(INCLUDEDIR)/adjlist.h  $(INCLUDEDIR)/grasp.h $(INCLUDEDIR)/Graph.h 
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/main.cpp -o $(OBJDIR)/main.o
$(OBJDIR)/experiments.o : $(SRCDIR)/experiments.cpp $(INCLUDEDIR)/ranking.h $(INCLUDEDIR)/new_ilp_solver.h  $(INCLUDEDIR)/old_ilp_solver.h $(INCLUDEDIR)/old_lp_solver.h $(INCLUDEDIR)/triple_solver.h $(INCLUDEDIR)/all_triple_formul_solver.h
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/experiments.cpp -o $(OBJDIR)/experiments.o
$(OBJDIR)/ranking.o: $(SRCDIR)/ranking.cpp $(INCLUDEDIR)/Functions.h $(INCLUDEDIR)/map_data_structure.h 
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/ranking.cpp -o $(OBJDIR)/ranking.o

$(OBJDIR)/new_ilp_solver.o: $(SRCDIR)/new_ilp_solver.cpp $(INCLUDEDIR)/ranking.h
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/new_ilp_solver.cpp -o $(OBJDIR)/new_ilp_solver.o

$(OBJDIR)/old_ilp_solver.o: $(SRCDIR)/old_ilp_solver.cpp $(INCLUDEDIR)/ranking.h $(INCLUDEDIR)/Functions.h $(INCLUDEDIR)/pair_search_tree.h
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/old_ilp_solver.cpp -o $(OBJDIR)/old_ilp_solver.o

$(OBJDIR)/pair_search_tree.o: $(SRCDIR)/pair_search_tree.cpp $(INCLUDEDIR)/ranking.h $(INCLUDEDIR)/Functions.h 
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/pair_search_tree.cpp -o $(OBJDIR)/pair_search_tree.o

$(OBJDIR)/heuristic_solver.o: $(SRCDIR)/heuristic_solver.cpp $(INCLUDEDIR)/ranking.h $(INCLUDEDIR)/Functions.h 
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/heuristic_solver.cpp -o $(OBJDIR)/heuristic_solver.o


$(OBJDIR)/grasp.o: $(SRCDIR)/grasp.cpp $(INCLUDEDIR)/ranking.h $(INCLUDEDIR)/Functions.h 
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/grasp.cpp -o $(OBJDIR)/grasp.o


$(OBJDIR)/primal_dual.o: $(SRCDIR)/primal_dual.cpp $(INCLUDEDIR)/ranking.h $(INCLUDEDIR)/Functions.h  $(INCLUDEDIR)/AdjList.h 
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/primal_dual.cpp -o $(OBJDIR)/primal_dual.o

$(OBJDIR)/triple_solver.o: $(SRCDIR)/triple_solver.cpp $(INCLUDEDIR)/ranking.h
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/triple_solver.cpp -o $(OBJDIR)/triple_solver.o

$(OBJDIR)/all_triple_formul_solver.o: $(SRCDIR)/all_triple_formul_solver.cpp $(INCLUDEDIR)/ranking.h $(INCLUDEDIR)/Functions.h 
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/all_triple_formul_solver.cpp -o $(OBJDIR)/all_triple_formul_solver.o

$(OBJDIR)/old_lp_solver.o: $(SRCDIR)/old_lp_solver.cpp $(INCLUDEDIR)/ranking.h
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/old_lp_solver.cpp -o $(OBJDIR)/old_lp_solver.o

$(OBJDIR)/Functions.o: $(SRCDIR)/Functions.cpp
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/Functions.cpp -o $(OBJDIR)/Functions.o

$(OBJDIR)/EdgeManager.o: $(SRCDIR)/EdgeManager.cpp $(INCLUDEDIR)/master_header.h
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/EdgeManager.cpp -o $(OBJDIR)/EdgeManager.o

$(OBJDIR)/CycleManager.o: $(SRCDIR)/CycleManager.cpp $(INCLUDEDIR)/master_header.h
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/CycleManager.cpp -o $(OBJDIR)/CycleManager.o

$(OBJDIR)/Graph.o: $(SRCDIR)/Graph.cpp $(INCLUDEDIR)/master_header.h
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/Graph.cpp -o $(OBJDIR)/Graph.o

$(OBJDIR)/adjlist.o: $(SRCDIR)/adjlist.cpp $(INCLUDEDIR)/master_header.h
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/adjlist.cpp -o $(OBJDIR)/adjlist.o

$(OBJDIR)/map_data_structure.o: $(SRCDIR)/map_data_structure.cpp
	$(CCC) -c $(CCFLAGS) $(SRCDIR)/map_data_structure.cpp -o $(OBJDIR)/map_data_structure.o

#$(OBJDIR)/environment.o: $(SRCDIR)/environment.cpp
#	$(CCC) -c $(CCFLAGS) $(SRCDIR)/environment.cpp -o $(OBJDIR)/environment.o

# Local Variables:
# mode: makefile
# End:
