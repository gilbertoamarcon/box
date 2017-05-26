#ifndef CLOSED_H
#define CLOSED_H
#include <string>
#include <unordered_map>
#include "../include/State.hpp"

using std::string;
using std::unordered_map;

class Closed{

	private:

		unordered_map<string, State*> map;

	public:

		Closed();

		virtual ~Closed();

		void insert(State *state);

		bool find(State *state);

};

#endif
