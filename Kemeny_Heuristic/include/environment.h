#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

/**
 * Environment holds all global shared state. It points to root problem.
 */

#include "master_header.h"
#include "ranking.h"

class Ranking;
class Environment {
public:
    /**
     * Constructor
     */
    Environment();

    virtual ~Environment();
public:

   /**
     * Input file name to read problem from
     */
    std::string file_name;

    /**
     * Name of the output log file
     */
    std::string output_file ;

   private:

    /**
     * Output stream for writing the tree
     */
    std::ostream* tree_ostream;

    /**
     * Output stream for writing the brief execution trace
     */
    std::ostream* trace_ostream;

    /**
     * Output stream for /dev/null. Useful for supressing outputs
     */
    std::ofstream dev_null;
};

#endif

