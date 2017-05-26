#include "../include/Open.hpp"

Open::Open(){}

Open::~Open(){
	while(!ordered.empty()){
		map.erase(ordered.back()->to_str());
		delete ordered.back();
		ordered.pop_back();
	}
}

void Open::insert(State *state){

	// Check if in the ordered vector
	unordered_map<string, State*>::const_iterator existing = map.find(state->to_str());
	if(existing != map.end()){

		if(existing->second->g < state->g)
			// Existing state with lower cost: Noop
			delete state;
		else
			// Existing state with higher cost: Substitute
			(*existing->second) = (*state);

		return;
	}

	// Not in the vector
	// Insert ordered using binary search
	int m = 0;
	int l = 0;
	int r = ordered.size()-1;
	for(;;){
		if(l >= r) break;
		m = floor((l+r)/2);
		if(ordered.at(m)->f == state->f) break;
		if(ordered.at(m)->f < state->f)
			r = m-1;
		else
			l = m+1;
	}
	ordered.insert(ordered.begin()+m,state);
	map[state->to_str()] = state;
}

State* Open::pop(){
	State *state = ordered.back();
	ordered.pop_back();
	map.erase(state->to_str());
	return state;
}

bool Open::empty(){
	return ordered.empty();
}