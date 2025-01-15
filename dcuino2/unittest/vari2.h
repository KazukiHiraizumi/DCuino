#include "invert2x2_c.h"
#include "invert3x3_c.h"

static int variation(int tspan,int cutoff,int scale,int base=0,int para=0){
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

  float sn0v=0,sn1v=0,sn2v=0;
  float sn0=vsamp,sn1=0,sn2=0,sn3=0,sn4=0;
  for(int i=0;i<vsamp;i++){
    float n=i;
    float n2=n*n;
    float n3=n2*n;
    float n4=n3*n;
    sn0v+=pv[i].beta;
    sn1v+=pv[i].beta*n;
    sn2v+=pv[i].beta*n2;
    sn1+=n;
    sn2+=n2;
    sn3+=n3;
    sn4+=n4;
  }
  float a_coef=0;
  if(para==0){  //  v=a*n+b
    float src[]={ sn2,sn1, sn1,sn0 };
    float inv[2*2];
    invert2x2(src,inv);
    a_coef=(inv[0]*sn1v+inv[1]*sn0v)*sn0;
  }
  else{
    float src[]={ sn4,sn3,sn2, sn3,sn2,sn1, sn2,sn1,sn0 };
    float inv[3*3];
    invert3x3(src,inv);
    a_coef=(inv[0]*sn2v+inv[1]*sn1v+inv[2]*sn0v)*sn0*sn0;
  }
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
  float atn=fabs(a_coef)*cutoff;
//  return sc*sigv*(atn<10000? (10000-atn)/10000:0);
  return sc*xsig*exp(-atn*atn/1e11);
}

