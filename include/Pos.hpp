#ifndef POS_H
#define POS_H
#include <string>
#include <vector>

using std::string;
using std::vector;

// Position structure
class Pos{

	public:

		int i;
		int j;

		// Compare two position vectors
		//  1: sta > state
		//  0: sta = state
		// -1: sta < state
		static int compare_vec(vector<Pos> &veca,vector<Pos> &vecb);

		// Manhattan distance between pos a and pos b
		static int manhattan(Pos a, Pos b);

		// Compares two positions
		static bool compare(Pos a, Pos b);
		
		// Position constructor
		Pos(int i, int j);

		// Position destructor
		virtual ~Pos();

		// Return string representation of position
		void to_str(char *buffer);
};

#endif
