#ifndef NOEFH
#define NOEFH

#include "napprox.h"

namespace noef{  //the Nth Order Equation Filter
  int variation(int *dat,int samp){
    float sig=0;
    for(int i=0;i<samp;i++){
      int d=dat[i];
      sig+=d*d;
    }
    return sqrt(sig/samp);
  }

  int N(int nsamp,int dim){
    logger::ALOG *p1=logger::data+logger::length()-1;  //tail of data
    logger::ALOG *p0=logger::data;   //head of data
    if(p1<p0+nsamp) return -1;
    logger::ALOG *pv=p1-nsamp;

    int cval[200],cfit[200];
    for(int i=0;i<nsamp;i++){
      cval[i]=pv[i].beta;
    }
    float coef[10];
    approx(dim,cval,nsamp,coef);
    auto neq=[&](float x){
      float y=0;
      for(int i=dim-1;i>=0;i--) y=y*x+coef[i];
      return y;
    };
    for(int i=0;i<nsamp;i++){
      int ccen=neq(i);
#ifdef ONE_SPAN_TEST
      printf("%lu %d %d %d %d %d %d\n",pv[i].stamp,pv[i].beta,pv[i].sigma,pv[i].duty,cval[i],ccen,cval[i]-ccen);
#endif
      cval[i]-=cfit[i]=ccen;
    }
    float sig=variation(cval,nsamp);
    return sig;
  }
  int analyze(int samp,int dim){
    if(samp<1000)
      return N(samp,dim);

    logger::ALOG *p1=logger::data+logger::length()-1;  //tail of data
    logger::ALOG *p0=logger::data;   //head of data
    int ts0=p1->stamp;
    int vsamp=0;
    for(;;p1--){
      if(p1<=p0) return -1;
      if(ts0-p1->stamp>samp) break;
      vsamp++;
    }
    if(vsamp>1) return N(vsamp,dim);
    else return -1;
  }
}

#endif
