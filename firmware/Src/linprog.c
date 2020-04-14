#include <linprog.h>

const int dimY=dim_y;
const int numCon=num_con;

float cost[dim_y+1];
float A[num_con][dim_y+1];
float x[num_con];
int B[num_con]={0,1,2};
int N[dim_y]={3};
float y[dim_y];



void presolve()
{
  for (int i=0; i<num_con; i++){
    pivot(i, dim_lambda+i);
  }
}

int linprog_solve()
{
  presolve();
  int status=0;
  for(int step=0; step<max_steps; step++){
    status=simplex_step();
    if (status!=0) break;
  }
  getSolution();
  return status;
}

void pivot(int pivotCon, int enterVar)
{
  //normalize con-th constraint with 1.0 in front of the var-th variable
  
  float divider=A[pivotCon][enterVar];
  if (divider!=1.0f){
    for (int j=0; j<=dimY; j++){         //j = column
      A[pivotCon][j] /= divider;
    }   
    A[pivotCon][enterVar]=1.0f;  
  }
  
  if (cost[enterVar]!=0.0f){
    float multiplier=cost[enterVar];
    for (int j=0; j<=dimY; j++){
      cost[j] -= multiplier * A[pivotCon][j];
    }
    cost[enterVar]=0.0f;
  }
  
  for (int i=0; i<numCon; i++){       //i = row, j = column
    if (i!=pivotCon){
      if (A[i][enterVar]!=0.0f){
        float multiplier=A[i][enterVar];
        for (int j=0; j<=dimY; j++){
          A[i][j] -= multiplier * A[pivotCon][j];
        }
        A[i][enterVar]=0.0f;
      }
    }
  }
}

int simplex_step()
{
  int enterVar = -1;
  int enterIdx = -1;
  
  float min = INFINITY;
  
  for (int idx=0;idx<dimY;idx++){
    int var=N[idx];
    if ((var!=-1) && (cost[var]<0) && (var<dim_lambda)){
      if (cost[var] < min){
        min = cost[var];
        enterVar = var;
        enterIdx = idx;
      }
    }
  }
  
  if (enterVar == -1) return 1; //1 if solution is optimal
  
  float minRatio = INFINITY;
  int pivotCon = -1;
  
  for (int con=0;con<numCon;con++){
    if (A[con][enterVar] > 0){
      float ratio = A[con][dimY] / A[con][enterVar];
      if (ratio < minRatio){
        minRatio = ratio;
        pivotCon = con;
      }
    }
  }
  
  if (isinf(minRatio)) return -1; // -1 if solution is unbounded
  
  pivot(pivotCon, enterVar);
  int temp=B[pivotCon];
  B[pivotCon]=N[enterIdx];
  N[enterIdx]=temp;
  return 0; //ok
}

void getSolution()
{
  for(int idx=0; idx<dimY; idx++){
    y[idx]=0;
  }
  for(int i=0; i<numCon; i++){
    y[B[i]]=A[i][dimY];
  }
}



