#ifndef APPROXH
#define APPROXH

#if APPROX==2
#include "invert3x3_c.h"
#elif APPROX==3
#include "invert4x4_c.h"
#else
#include "invert2x2_c.h"
#endif

static void approx(int *dat,int samp,float *ans){
  float sn0v=0,sn1v=0,sn2v=0,sn3v=0;
  float sn0=samp,sn1=0,sn2=0,sn3=0,sn4=0,sn5=0,sn6=0;
  for(int i=0;i<samp;i++){
    float n=i;
    float n2=n*n;
#if APPROX>=2
    float n3=n2*n;
    float n4=n3*n;
#endif
#if APPROX>=3
    float n5=n4*n;
    float n6=n5*n;
#endif
    sn0v+=dat[i];
    sn1v+=dat[i]*n;
    sn1+=n;
    sn2+=n2;
#if APPROX>=2
    sn2v+=dat[i]*n2;
    sn3+=n3;
    sn4+=n4;
#endif
#if APPROX>=3
    sn3v+=dat[i]*n3;
    sn5+=n5;
    sn6+=n6;
#endif
  }
#if APPROX==2
  float src[]={ sn4,sn3,sn2, sn3,sn2,sn1, sn2,sn1,sn0 };
  float inv[3*3];
  invert3x3(src,inv);
  ans[0]=(inv[0]*sn2v+inv[1]*sn1v+inv[2]*sn0v);
  ans[1]=(inv[3]*sn2v+inv[4]*sn1v+inv[5]*sn0v);
  ans[2]=(inv[6]*sn2v+inv[7]*sn1v+inv[8]*sn0v);
#elif APPROX==3
  float src[]={ sn6,sn5,sn4,sn3, sn5,sn4,sn3,sn2, sn4,sn3,sn2,sn1, sn3,sn2,sn1,sn0 };
  float inv[4*4];
  invert4x4(src,inv);
  ans[0]=(inv[0]*sn3v+inv[1]*sn2v+inv[2]*sn1v+inv[3]*sn0v);
  ans[1]=(inv[4]*sn3v+inv[5]*sn2v+inv[6]*sn1v+inv[7]*sn0v);
  ans[2]=(inv[8]*sn3v+inv[9]*sn2v+inv[10]*sn1v+inv[11]*sn0v);
  ans[3]=(inv[12]*sn3v+inv[13]*sn2v+inv[14]*sn1v+inv[15]*sn0v);
#else
  float src[]={ sn2,sn1, sn1,sn0 };
  float inv[2*2];
  invert2x2(src,inv);
  ans[0]=(inv[0]*sn1v+inv[1]*sn0v);
  ans[1]=(inv[2]*sn1v+inv[3]*sn0v);
#endif
}

#endif
