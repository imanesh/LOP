#include "environment.h"

Environment::Environment() {
    // Open /dev/null stream
    dev_null.open("/dev/null", std::ofstream::out);
    // Redirect stream to /dev/null
    tree_ostream = &dev_null;
    trace_ostream = &dev_null;
    // Initialize t_strart. Note that actual t_start is initialized in bnb.cpp
}

Environment::~Environment() {
    dev_null.close();
}

