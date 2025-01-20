#include "acoef.h"

static int sig(int x,int y){
  int n=x/5000;
  return (n&1)? -y:y;
}

static int intensity(logger::ALOG *pv,int samp,int nwav,int phase,int cutoff){
  int w=100*nwav/samp;
  int ph=100*phase-w;
  int xsig=0;
  for(int i=1;i<samp;i++){
    int der=(pv[i].beta-pv[i-1].beta);
//    int der=(pv[i].beta-pv[i-1].beta)*10000/pv[i].interval;
    if(der>cutoff) der=cutoff;
    else if(der<-cutoff) der=-cutoff;
    xsig+=sig(w*i+ph,der);
  }
  return abs(100*xsig/(samp-1));
}
static int vari5(int tspan,int period,int cutoff){
  logger::ALOG *p1=logger::data+logger::length()-1;  //tail of data
  logger::ALOG *p0=logger::data;   //head of data
  logger::ALOG *pv;
  int ts0=p1->stamp;
  int vsamp=0;
  for(auto p=p1;;p--){
    if(p<=p0) return -1;
    int tsamp=ts0-p->stamp;
    if(tsamp>tspan) break;
    pv=p;
    vsamp++;
  }

  int xmax=0;
  for(int i=0;i<=5;i++){
    float x=intensity(pv,vsamp,period,50/5*i,cutoff);
    if(xmax<x) xmax=x;
  }
  return xmax;
}

