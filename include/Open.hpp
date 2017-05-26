#ifndef OPEN_H
#define OPEN_H
#include <string>
#include <vector>
#include <unordered_map>
#include "../include/State.hpp"

using std::string;
using std::vector;
using std::unordered_map;

class Open{

	private:

		vector<State*> ordered;

		unordered_map<string, State*> map;

	public:

		Open();

		virtual ~Open();

		void insert(State *state);

		State* pop();

		bool empty();

};

#endif
