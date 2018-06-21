//============================================================================
// Name        : Kemeny_project.cpp
// Author      : Ehsan
// Version     :
// Copyright   : Your copyright notice
//============================================================================



#include "experiments.h"

int main(int arg, char** argv)
{
	// Install handler for SIGINT
   // signal(SIGINT, HandleIntSignal);
	//Declare a CLI handler
    TCLAP::CmdLine cli("", ' ', "");
	// Input file name argument
	TCLAP::ValueArg<std::string> input_file_name_arg("f", "input-file",
                          "Path of input file containing problem",
                                      false, "", "string", cli);
    //--------------------------------------------------------------------------
    // Run CPLEX options
    //--------------------------------------------------------------------------
    TCLAP::SwitchArg ip("", "ip", "Run cplex-ip on the input problem", cli);
    //TCLAP::SwitchArg verbose("", "verbose", "Turn on verbose mode", cli);
    TCLAP::SwitchArg lp("", "lp", "Run cplex-lp on the input problem", cli);
    //--------------------------------------------------------------------------
    // Run Heuristic  options
    //--------------------------------------------------------------------------
    
	TCLAP::SwitchArg mip_heur("", "mip_heur", "Run mip heuristic on the input problem", cli);
    TCLAP::SwitchArg grasp("", "grasp", "Run grasp on the input problem", cli);
	TCLAP::ValueArg<int> grasp_construction_method("", "grasp_construction_method",
			            "method number to contsruct the initial solution ",
						            false, 1, "int", cli);
	TCLAP::ValueArg<float> grasp_alpha("a", "grasp_alpha",
			            "alpha for RCL set ngrasp ",
						            false, 0.7, "float", cli);
	TCLAP::SwitchArg experiment("", "experiment", "Run experiment on mip heuristic on the input problem", cli);

	bool success = true;
	try {
		// Parse CLI
		cli.parse(arg, argv);
	} catch (...) {
		success = false;
	}

	// Handle mistakes in CLI arguments
	if (!success || arg == 1) {
		std::cout << "For complete usage and help type:\n     kemeny --help\n";
		return 1;
	}
	std::string file_name = input_file_name_arg.getValue();
	cout << "file name " << file_name << endl;
	const char * s = file_name.c_str();
	cout << "arg = "  << arg << endl;

	if (ip.getValue()) {
		if (file_name.empty()) {
			std::cout << "Error: file name cannot be empty" << std::endl;
			return 0;
		}
		return 0;
	}
	if (lp.getValue()) {
		if (file_name.empty()) {
			std::cout << "Error: file name cannot be empty" << std::endl;
			return 0;
		}
		return 0;
	}
	if(experiment.getValue()){
		if (file_name.empty()) {
			std::cout << "Error: file name cannot be empty" << std::endl;
			return 0;
		}
		experiments(s);
	}

	if(mip_heur.getValue()){
		if (file_name.empty()) {
			std::cout << "Error: file name cannot be empty" << std::endl;
			return 0;
		}
		mip_heuristic(s);
	//	RunIterativeMIP(s);
	}
	if(grasp.getValue()){
		if (file_name.empty()) {
			std::cout << "Error: file name cannot be empty" << std::endl;
			return 0;
		}
		float alpha = grasp_alpha.getValue();
		int gsp_cons_method = grasp_construction_method.getValue();
		RunGrasp(s, gsp_cons_method, alpha);
	}

	return 0;
}  
