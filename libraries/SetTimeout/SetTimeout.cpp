#include "Arduino.h"
#include "SetTimeout.h"

#ifdef INC_FREERTOS_H
#include <freertos/FreeRTOS.h>
SemaphoreHandle_t xMutex = NULL;
inline void lock_init(){
	xMutex = xSemaphoreCreateMutex();
}
inline void lock_on(){
	const TickType_t xTicksToWait=1000UL;
	BaseType_t xStatus = xSemaphoreTake(xMutex, xTicksToWait);
}
inline void lock_off(){
	xSemaphoreGive(xMutex);	
}
#endif
#ifdef ARDUINO_ARCH_MBED
#include <rtos.h>
static rtos::Mutex mutex;
inline void lock_init(){
}
inline void lock_on(){
  mutex.lock();
}
inline void lock_off(){
  mutex.unlock();
}
#endif

struct SetTimeoutTab{
  long timeout;
  uint8_t msg[8];
  int msglen;
  SetTimeoutCallback func;
  SetTimeoutTab();
};

SetTimeoutTab::SetTimeoutTab(){
  func=NULL;
  timeout=0;
  memset(msg,0,sizeof(msg));
  msglen=0;
}

SetTimeoutClass::SetTimeoutClass(void){
  tbl=new SetTimeoutTab[20];
  nqu=0;
  lock_init();
}
void SetTimeoutClass::tabshift(SetTimeoutTab *t,int blocks,bool ins){
  if(blocks<=0) return;
  int len=blocks*sizeof(SetTimeoutTab);
  unsigned char buf[20*sizeof(SetTimeoutTab)];
  if(ins){
    memcpy(buf,t,len);
    memcpy(t+1,buf,len);
  }
  else{
    memcpy(buf,t+1,len);
    memcpy(t,buf,len);
  }
}
long SetTimeoutClass::set(SetTimeoutCallback f,int ms){
  return set(NULL,-1,(SetTimeoutCallbackPN)f,ms);
}
long SetTimeoutClass::set(char *s,SetTimeoutCallbackP f,int ms){
  return set((uint8_t *)s,0,(SetTimeoutCallbackPN)f,ms);
}
long SetTimeoutClass::set(uint8_t *s,int l,SetTimeoutCallbackPN f,int ms){
  lock_on();
  long now=micros();
  long tout=now+(((long)ms)<<10);
  SetTimeoutTab *t=tbl;
  for(int n=0;n<nqu;n++,t++){
    long dt=tout-t->timeout;
    if(dt<0){
      tabshift(t,nqu-n,true);  //insert tab
      break;
    }
  }
  t->func=(SetTimeoutCallback)f;
  t->timeout=tout;
  t->msglen=l;
  if(s!=NULL){
  	if(l==0) strcpy((char *)t->msg,(char *)s);
  	else memcpy(t->msg,s,l);
  }
  nqu++;
  lock_off();
  return tout;
}
int SetTimeoutClass::clear(long d){
  lock_on();
  int n=this->lookup(d);
  if(n>=0) tabshift(tbl+n,(--nqu)-n,false);  //delete tab
  lock_off();
  return n;
}
int SetTimeoutClass::lookup(long d){
  SetTimeoutTab *et=tbl;
  for(int n=0;n<nqu;n++,et++){
    if(et->timeout==d) return n;
  }
  return -1;
}
SetTimeoutCallback SetTimeoutClass::spinOnce(void){
  if(nqu==0) return NULL;
  lock_on();
  SetTimeoutTab et=tbl[0];
  long now=micros();
  long diff=et.timeout-now;
  if(diff<0) tabshift(tbl,--nqu,false);
  else et.func=NULL;
  lock_off();
  if(et.func!=NULL){
    if(et.msglen>0) (*(SetTimeoutCallbackPN)et.func)(et.msg,et.msglen);
    else if(et.msglen==0) (*(SetTimeoutCallbackP)et.func)((char *)et.msg);
    else (*(SetTimeoutCallback)et.func)();
  }
  return et.func;
}

SetTimeoutClass setTimeout;
