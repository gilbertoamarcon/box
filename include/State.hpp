#ifndef STATE_H
#define STATE_H
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <stack>
#include <vector>
#include "../include/Map.hpp"
#include "../include/Pos.hpp"
#include "../include/utils.hpp"

using std::string;
using std::vector;
using std::stack;
using std::vector;

// State class definition
class State{

	public:

		// Loading map object
		static Map *map;

		// Start and end node
		static State *start;
		static State *goal;

		// Pointer to parent
		State *parent;

		// Cost to reach from start
		int g;

		// Estimate path cost
		int f;

		// Action vector taken from parent
		string action_vector;

		// Robots positions
		vector<Pos> robots;

		// Boxes positions
		vector<Pos> boxes;

		// Constructor
		State();

		// Constructor from parent
		State(State *parent,string action_vector);

		virtual ~State();

		// Compare with two states
		//  1: this > state
		//  0: this = state
		// -1: this < state
		static int compare(State *sta, State *stb);

		// World print
		static void display_world(State *state);

		// Heuristic distance to goal
		int heuristic(State *goal);

		// Write state representation to string
		string to_str();

		// Return true if state equals goal
		bool is_goal(State *goal);

		// Return stack with children states
		void expand(stack<State*> *children);

	private:

		// Buffer for printing
		char print_buffer[BUFFER_SIZE];

		// Recursive action vector expansion
		void expand_action_vector(string action_vector, int i, char *action, stack<State*> *children);
		
		// Validate state against world rules
		bool validate();

		// Check if a set of box coordinates would result in a deadlock
		bool is_Deadlock(int i, int j);
};

#endif
