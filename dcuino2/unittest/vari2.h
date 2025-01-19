#include "acoef.h"

static int vari2(int tspan,int cutoff,int scale,int base=0,int order=2){
  logger::ALOG *p1=logger::data+logger::length()-1;  //tail of data
  logger::ALOG *p0=logger::data+base;   //head of data
  logger::ALOG *pv;
  int ts0=p1->stamp;
  int vsamp=0;
  int vmean=0;
  for(auto p=p1;;p--){
    if(p<=p0) return -1;
    int tsamp=ts0-p->stamp;
    if(tsamp>tspan) break;
    pv=p;
    vmean+=p->beta;
    vsamp++;
  }
  vmean/=vsamp;

  float xmean=0;
  for(int i=1;i<vsamp;i++){
    float er=pv[i].beta-pv[i-1].beta;
    xmean+=er;
  }
  xmean=xmean/(vsamp-1);
  float xsig=0;
  for(int i=1;i<vsamp;i++){
    float er=pv[i].beta-pv[i-1].beta-xmean;
    xsig+=er*er;
  }
  xsig=sqrt(xsig/(vsamp-1));

  float sc=1.0/(1<<(scale%100));
  float a1=fabs(acoef(pv,vsamp/2,order));
  float a2=fabs(acoef(pv+vsamp/4,vsamp/2,order));
  float a3=fabs(acoef(pv+vsamp/2,vsamp/2,order));
  float atn=(a1>a2? a1:a2>a3? a2:a3)*cutoff;
//  return sc*sigv*(atn<10000? (10000-atn)/10000:0);
  return sc*xsig*exp(-atn*atn/1e11);
}

