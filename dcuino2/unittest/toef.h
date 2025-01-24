#ifndef TOEFH
#define TOEFH

#ifdef APPROX
#undef APPROX
#endif

#define APPROX 3
#include "approx.h"

namespace toef{  //the Third Order Equation Filter
  int variation(int *dat,int samp){
    float sig=0;
    for(int i=0;i<samp;i++){
      int d=dat[i];
      sig+=d*d;
    }
    return sqrt(sig/samp);
  }

  int N(int nsamp,int cutoff){
    logger::ALOG *p1=logger::data+logger::length()-1;  //tail of data
    logger::ALOG *p0=logger::data;   //head of data
    if(p1<p0+nsamp) return -1;
    logger::ALOG *pv=p1-nsamp;

    int cval[200];
    for(int i=0;i<nsamp;i++){
      cval[i]=pv[i].beta;
    }
    float coef[4];
    approx(cval,nsamp,coef);
    auto toe=[&](float x){ return ((coef[0]*x+coef[1])*x+coef[2])*x+coef[3];};
    for(int i=0;i<nsamp;i++){
      int ccen=toe(i);
#ifdef ONE_SPAN_TEST
      printf("%lu %d %d %d %d %d %d\n",pv[i].stamp,pv[i].beta,pv[i].sigma,pv[i].duty,cval[i],ccen,cval[i]-ccen);
#endif
      cval[i]-=ccen;
    }
    float sig=variation(cval,nsamp);
    float a=3*coef[0];
    float b=2*coef[1];
    float c=coef[2];
    float D=b*b-4*a*c;
    if(D>0){
      float sqD=sqrt(D);
      float x1=(-b+sqD)/2/a;
      float x2=(-b-sqD)/2/a;
      float y1=toe(x1);
      float y2=toe(x2);
      float w=fabs(y2-y1)*nsamp/fabs(x2-x1)*cutoff/1e5;
      return sig*exp(-w*w);
    }
    else return sig;
  }

  int analyze(int samp,int cutoff){
    if(samp<1000)
      return N(samp,cutoff);

    logger::ALOG *p1=logger::data+logger::length()-1;  //tail of data
    logger::ALOG *p0=logger::data;   //head of data
    int ts0=p1->stamp;
    int vsamp=0;
    for(;;p1--){
      if(p1<=p0) break;
      if(ts0-p1->stamp>samp) break;
      vsamp++;
    }
    if(vsamp>1) return N(vsamp,cutoff);
    else return -1;
  }
}

#endif
