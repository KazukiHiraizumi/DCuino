#include <stdio.h>
#include <stdlib.h>
#include <cmath>

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;
/*typedef char int8_t;
typedef short int16_t;
typedef long int32_t;*/

namespace logger{
  struct ALOG{
    uint32_t stamp;
    uint8_t mode;
    uint8_t duty;
    uint8_t cmd;
    uint16_t interval;
    uint16_t latency;
    uint16_t omega;
    int16_t beta;
    int16_t sigma;
    int16_t eval;
  };
  ALOG stage;
  ALOG *data;
  int size;
  int length(){ return size;}
}

logger::ALOG buffer[1000];

int max3(int a,int b,int c){
  if(a>b){
    return a>c? a:c;
  }
  else{
    return b>c? b:c;
  }
}

#define APPROX 3
#include "dft.h"
#include "toef.h"

int main(int argc,char **argv){
  FILE *f=fopen("log4.txt","r");
  char aline[256];
  char hdr[20];
  logger::data=buffer;
  logger::size=1;
  logger::data[0].stamp=0;
  for(;;logger::size++){
    fgets(aline,255,f);
    logger::ALOG* a=logger::data+logger::size;
    auto ret=sscanf(aline,"%s %hu %c %hu %hu %hd %hd %c %c",
      hdr,
      &a->interval,
      &a->mode,
      &a->latency,
      &a->omega,
      &a->beta,
      &a->eval,
      &a->duty,
      &a->cmd);
    if(ret==EOF) break;
    a->stamp=(a-1)->stamp+a->interval;
//    int f1=lbft::analyze(300000,200,2000)*3>>2;
//    int g1=lbft::analyze(300000,230,2000)*3>>2;
//    int d1=lbft::analyze(300000,270,2000)*3>>2;
    int f1=dft::analyze(200000,66)*3>>2;
//    int g1=dft::analyze(200000,66)*3>>2;
//    int d1=dft::analyze(200000,44)*3>>2;
//    int f1=toef::analyze(200000,200)*3>>2;
    int g1=toef::analyze(200000,20)*3>>2;
    int d1=toef::analyze(200000,50)*3>>2;
    printf("%lu %d %d %d %d %d %d\n",a->stamp,a->beta,a->sigma,a->duty,f1,g1,d1);
  }
  fclose(f);

  return 0;
}
