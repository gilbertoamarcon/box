#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <ctime>

void tic();

void toc();

double t_start;
double t_end;
double t_sum;

void tic(){
	t_start = (double)(clock());
}

void toc(){
	t_end = (double)(clock()) - t_start;
	printf("Time since last tic: %f \n", t_end / (double)CLOCKS_PER_SEC);
}

void tic_add(){
	t_sum += (double)(clock()) - t_start;
	t_start = (double)(clock());
}

void toc_sum(){
	printf("Total time of tic_adds: %f \n", t_sum / (double)CLOCKS_PER_SEC);
}