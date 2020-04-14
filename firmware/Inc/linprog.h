#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

  
#define dim_x           4
#define dim_lambda      16
#define dim_y           21
#define num_con         5
#define max_steps       50

extern float A[num_con][dim_y+1];
extern float cost[dim_y+1];
extern int B[num_con];
extern int N[dim_y];
extern float y[dim_y];

void pivot(int, int);
void presolve();
void getSolution();
int linprog_solve(void);
int simplex_step();
//int update_basic(int, int, int);
int add_basic(void);