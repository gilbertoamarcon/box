#include "../include/Closed.hpp"

Closed::Closed(){}

Closed::~Closed(){
	unordered_map<string, State*>::iterator it = map.begin();
	for(it = map.begin(); it != map.end(); it++)
		delete it->second;
}

void Closed::insert(State *state){
	map[state->to_str()] = state;
}

bool Closed::find(State *state){
	return map.find(state->to_str()) != map.end();
}