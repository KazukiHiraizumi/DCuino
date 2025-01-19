#ifndef ACOEFH
#define ACOEFH

#include "invert2x2_c.h"
#include "invert3x3_c.h"

static float acoef(logger::ALOG *pv,int samp,int order=2){
  float sn0v=0,sn1v=0,sn2v=0;
  float sn0=samp,sn1=0,sn2=0,sn3=0,sn4=0;
  for(int i=0;i<samp;i++){
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
  if(order==1){  //  v=a*n+b
    float src[]={ sn2,sn1, sn1,sn0 };
    float inv[2*2];
    invert2x2(src,inv);
    return (inv[0]*sn1v+inv[1]*sn0v)*sn0;
  }
  else{
    float src[]={ sn4,sn3,sn2, sn3,sn2,sn1, sn2,sn1,sn0 };
    float inv[3*3];
    invert3x3(src,inv);
    return (inv[0]*sn2v+inv[1]*sn1v+inv[2]*sn0v)*sn0*sn0;
  }
}

#endif
