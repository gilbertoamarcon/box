#ifndef MAP_H
#define MAP_H
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../include/Pos.hpp"

using std::vector;

// File Parameters
#define BUFFER_SIZE		256

class Map{

	public:
		int coordinate2index(int i,int j);
		void index2coordinate(int index,int *i,int *j);
		int cols;
		int rows;
		int *map;
		vector<Pos> corners;
		vector<Pos> deadlocks;
		Map();
		Map(int cols, int rows, int *data);
		virtual ~Map();
		int get_value(int i, int j);
		bool is_Corner(int i, int j);
		bool is_Deadlock(int i, int j);
		bool evaluate_Corner_Pair(Pos c1, Pos c2, vector<Pos> goals);
		void set_Corners(vector<Pos> goals);
		void set_Deadlocks(vector<Pos> goals);


};

#endif
