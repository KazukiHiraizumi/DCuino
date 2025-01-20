#include "acoef.h"

static float sig(float x){
  int n=floor(x/M_PI);
  return (n&1)? -1:1;
}

static float intensity(logger::ALOG *pv,int samp,float nwav,float phase,float cutoff){
  float w=2*M_PI*nwav/samp;
  float ph=2*M_PI*phase-w;
  float xsig=0;
  for(int i=1;i<samp;i++){
    float der=(pv[i].beta-pv[i-1].beta);//*1.0e3/pv[i].interval;
    if(der>cutoff) der=cutoff;
    else if(der<-cutoff) der=-cutoff;
    float m=sig(w*i+ph);
    xsig+=der*m;
  }
  return fabs(xsig/(samp-1));
}
static float vari4(int tspan,float period,float cutoff){
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

  float xmax=0;
  for(int i=0;i<=5;i++){
    float x=intensity(pv,vsamp,period,0.5/5*i,cutoff);
    if(xmax<x) xmax=x;
  }
  return xmax;
  float atn=fabs(acoef(pv,vsamp,2))*cutoff;
  return xmax*exp(-atn*atn/1e11);
}

