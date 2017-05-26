#ifndef SEARCH_H
#define SEARCH_H
#include <ctime>
#include <stack>
#include "../include/Open.hpp"
#include "../include/Closed.hpp"

using std::stack;

// Searchition structure
class Search{

	public:
	
		// Search results
		static int num_exp_nodes;
		static double planning_time;
	
		// Search parameters
		static int max_iterations;
		static float time_lim_secs;
		static float epsilon;

		// Plan
		static stack<State> plan;

		// Loading configuration from file
		static void load_search_parameters(char *filename);

		// Print plan
		static void print_plan();

		// Store plan execution history
		static void store_plan(char *filename);

		// Search for a plan
		static int search();

	private:

		// Insert child to open vector if correct conditions met
		static void new_child(State *child, Open *open, Closed *closed);
};

#endif
