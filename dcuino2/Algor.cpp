#include <Arduino.h>
#include <SetTimeout.h>
#include "Algor.h"
#include "Param.h"
#include "Logger.h"
#include "Dcore.h"
#include "Ble.h"

#include "unittest/vari2.h"

//params
uint8_t algor_param[]={
  0,0,0,0,  130,100,100,0,
  20,30,10,30,  20,17,50,10,
  5,0,10,20,  30,8,6,5,
  0,155,13,159,  27,145,45,101,
  79,69,128,39,  189,23,255,19,
  0,50,10,50,  10,100,30,100,
  255,255,255,255,  255,255,255,255
};

//elapsed time
static uint32_t tusec;
static uint16_t revs;
//observer vars
static float wmax,wh,bh,hcoef1,hcoef2,bf,wro;
//profile
static uint8_t iflag,iprof,ilite;
static int16_t ivmax;
static float ibbase;
//controls
static uint8_t zflag;
static float z1integ,z2integ;
static void (*zfunc)();
static int16_t zfref;
//table
static uint8_t tbl_index;
//Ocillation analyzer
#define FFTN 32  //samples
static int32_t fvalue;
static void (*ffunc)();
static uint16_t fdbase;
//Macros
#define MAX(a,b) ((a)>(b)? a:b)
#define MIN(a,b) ((a)<(b)? a:b)
#define ARRSZ(a) (sizeof(a)/sizeof(a[0]))

/**************************************************************************/
static int satuate(int val,int lo =0, int hi =255){
  return MIN(hi,MAX(lo,val));
}
static int interp(int y1,int y2,int dx,int w){
  if(w<0) return y1;
  else if(w>dx) return y2;
  else return (y2*w+y1*(dx-w))/dx;
}
static int readTbl4k(int p,int w){
  auto x1=PRM_ReadData(p)<<4;
  auto x2=PRM_ReadData(p+2)<<4;
  uint8_t y1,y2;
  if(w<0) w=0;
  if(w>4095) w=4095;
  tbl_index=0;
  while(x2<w){
    x1=x2;
    tbl_index++;
    p+=2;
    x2=PRM_ReadData(p+2)<<4;
  }
  y1=PRM_ReadData(p+1);
  y2=PRM_ReadData(p+3);
  auto y=interp(y1,y2,x2-x1,w-x1);
  return MAX(0,y);
}
static int readTblN(int p,int b,int n){
  if(b<PRM_ReadData(p)) return PRM_ReadData(p+1);
  n=(n-1)<<1;
  if(b>PRM_ReadData(p+n)) return PRM_ReadData(p+n+1);
  return readTbl4k(p,b<<4);
}
static int readProf(int prof_tbl,int prof_tmsec,uint8_t &idx){
  auto hval=readTbl4k(prof_tbl,prof_tmsec);
  idx=tbl_index;
  return hval;
}
static int readProf(int prof_tbl,int idx){
  return PRM_ReadData(prof_tbl+(idx<<1)+1);
}
static void setPol(float polr,float poli){
  hcoef1=2*polr;
  hcoef2=polr*polr+poli*poli;
}
static int sigduty(int tspan){
  logger::ALOG *p1=logger::data+logger::length()-1;  //tail of data
  logger::ALOG *p0=logger::data;   //head of data
  for(int i=1,ts0=p1->stamp,sig=0;;i++,p1--){
    int rat=sig*100/i;
    if(p1<=p0) return rat;
    int tsamp=ts0-p1->stamp;
    if(tsamp>tspan) return rat;
    if(p1->sigma>0) sig++;
  }
}
void algor_prepare(){
  tusec=0;
}
uint16_t algor_update(int32_t dtu,int32_t otu){
  if(dtu==0) return 0;
//Measuring
  auto dt=dtu*1.0e-6;
  auto wrps=2*M_PI/dt;
  if(tusec==0){
    revs=iprof=iflag=zflag=0;
    wmax=wh=wro=wrps;
    bh=0;
    setPol(PRM_ReadData(5),0);
  }
  revs++;
  if(wmax<wrps) wmax=wrps;
  auto tmsec=(tusec+=dtu)/1000;
  auto uat=(float)otu/dtu*PRM_ReadData(4)/(8*M_PI); //duty/Tau
  uint8_t nloop=dtu/500;
  if(nloop<2) nloop=2;
  auto dtn=dt/nloop;
  auto bho=bh;
  for(int i=0;i<nloop;i++){
    auto ii=i+1;
    auto wi=(ii*wrps+(nloop-ii)*wro)/nloop;
    auto werr=wi-wh;
    auto db=werr*hcoef2;
    wh=wh+(werr*hcoef1+bh-wi*uat)*dtn;
    bh=bh+db*dtn;
  }
  wro=wrps;
  float dbh=(bh-bho)/dt;
  float sigma=(bh-(int)PRM_ReadData100x(15))+PRM_ReadData(14)*dbh/wrps;
//Logger
  logger::stage.stamp=tusec;
  logger::stage.omega=round(wrps);
  logger::stage.beta=satuate(round(bh),-32768,32767);
  logger::stage.sigma=satuate(round(sigma),-32768,32767);
  switch(PRM_ReadData(3)){
    case 0: logger::stage.eval=logger::stage.sigma; break;
    case 2: logger::stage.eval=satuate(iflag*20,0,255); break;
    case 4: logger::stage.eval=satuate(fvalue,0,255); break;
  }

//i-Block: base profile
  auto ivalue=readProf((24),tmsec,iprof);
  switch(iflag){
    case 0:
      iflag=1;
      ilite=0;
      ivmax=ivalue;
      ibbase=bh;
      fvalue=0;
      ffunc=NULL;
    case 1:
      if(ivmax<ivalue) ivmax=ivalue;
      if(ibbase<bh) ibbase=bh;
      if(wrps>PRM_ReadData10x(8)){
        iflag=2;
        break;
      }
      else if(tmsec>50 && bh<(int)PRM_ReadData100x(9)){
        iflag=4;
        ilite=1;    //light casting
        wh=wrps;
        bh=ibbase=0;
        setPol(PRM_ReadData(6),0);
        dcore::shift();  //RunLevel =>4
        break;
      }
      break;
    case 2:
      if(ivmax<ivalue) ivmax=ivalue;
      if(ibbase<bh) ibbase=bh;
      else if(bh<(int)PRM_ReadData100x(9)){
        iflag=3;
        ibbase=bh;
        dcore::shift();  //RunLevel =>4
      }
      break;
    case 3:
      if(ivmax<ivalue) ivmax=ivalue;
      if(ibbase>bh) ibbase=bh;
      else if(bh-ibbase>(int)PRM_ReadData100x(10) || tmsec>PRM_ReadData(11)){
        iflag=4;
        wh=wrps;
        bh=ibbase=0;
        setPol(PRM_ReadData(6),0);
      }
      break;
    case 4:
      if(ffunc==NULL){
        fdbase=logger::length();
        setTimeout.set(ffunc=[](){
          int span1=PRM_ReadData1000x(16);
//          int span2=PRM_ReadData1000x(17);
//          int step=PRM_ReadData1000x(18);
          int scale=PRM_ReadData(19);
          int cutoff=PRM_ReadData(20);
          int fc=variation(span1,cutoff,scale,fdbase,1);
          if(fc>=0) fvalue=fc;
          if(sigduty(PRM_ReadData1000x(12))<PRM_ReadData(13)) iflag=5;
          if(dcore::RunLevel>0){
            setTimeout.set(ffunc,20);
          }
        },PRM_ReadData(12));
      }
    case 5:
      if(PRM_ReadData(7)>0 && bh<-(int)PRM_ReadData100x(7)) iflag=6;
      break;
    case 6:
      return ivmax;
  }

  auto integfB= [&](float& integ,float ref){
    integ+=0.5*(bh-ref)*dt;
  };
  static auto integfF= [&](float& integ,float ref){
    integ+=0.05*(fvalue-ref);
  };
  int zcmd= ilite? ivalue*PRM_ReadData(42)/100:ivalue;
  switch(zflag){
    case 0:   //speed is low
      zflag=1;
      zfunc=NULL;
      z1integ=z2integ=0;  //integral
    case 1:
      zflag=iflag;
      zcmd=0;
      break;
    case 2:   //tension > thres
      zflag=iflag;
      zcmd=PRM_ReadData10x(40);   //overrun surpressor in 10usec
      break;
    case 3:   //down trend
      zflag=iflag;
      zcmd=zcmd*PRM_ReadData(41)/100;    //initial middle brake
      break;
    case 4:{  //Collision state(Sliding mode control)
      if(sigma>0){
        zcmd=ivmax*PRM_ReadData(43)/100;    //low brake
        integfB(z2integ,0);
      }
      else{
        int zmin=ivmax*PRM_ReadData(44)/100;
        zcmd-=zcmd*satuate(z2integ*(int)PRM_ReadData(45)/100,0,100)/100;
        if(zcmd<zmin) zcmd=zmin;
      }
      integfB(z1integ,PRM_ReadData100x(46));
      if(iflag>4){
        zflag=5;
        z1integ= z1integ>0? z1integ*PRM_ReadData(47)/100:0; //initial value valid ratio
        zfref=PRM_ReadData(49);  //fvalue reference
        setTimeout.set(zfunc=[](){
          integfF(z1integ,zfref);
          if(dcore::RunLevel>0){
            setTimeout.set(zfunc,20);
          }
        },20);
      }
      if(PRM_ReadData(3)==4) logger::stage.eval=satuate(z2integ,0,255);
      else if(PRM_ReadData(3)==5) logger::stage.eval=satuate(z1integ,0,255);
      break;
    }
    case 5:{ //Steady state(PI control)
      int zmin=ivmax*PRM_ReadData(48)/100;
      zcmd-=zcmd*satuate(z1integ*(int)PRM_ReadData(51)/100,0,100)/100;
      if(zcmd>0){
        zcmd-=zcmd*satuate((fvalue-zfref)*(int)PRM_ReadData(50)/100,0,100)/100;
      }
      if(zcmd<zmin) zcmd=zmin;
      if(PRM_ReadData(3)==5) logger::stage.eval=satuate(z1integ,0,255);
      break;
    }
  }
  return zcmd;
}
