#ifndef NAPPROXH
#define NAPPROXH

#include "lu.h"

#define MAXDIM 7

static void approx(int dim,int *dat,int samp,float *ans){
  float snv[MAXDIM];
  float snn[MAXDIM*2];
  for(int i=0;i<dim;i++) snn[2*i]=snn[2*i+1]=snv[i]=0;
  for(int i=0;i<samp;i++){
    float nn=1;
    float y=dat[i];
    for(int d=0;d<dim*2;d++,nn*=i){
      snn[d]+=nn;
      if(d<dim) snv[d]+=nn*y;
    }
  }
  float lu[MAXDIM*MAXDIM];
  float *tg=lu;
  for(int i=0;i<dim;i++){
    float *sc=snn+i;
    for(int j=0;j<dim;j++,tg++) *tg=sc[j];
  }
  int pivot[MAXDIM];
  LU_decomposition(dim, pivot, lu);
  LU_solver(dim, pivot, lu, snv, ans);
}

#endif
