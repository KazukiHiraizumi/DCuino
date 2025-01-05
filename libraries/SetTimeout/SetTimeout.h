#ifndef SetTimeout_h
#define SetTimeout_h

#include	"Arduino.h"

typedef void (*SetTimeoutCallback)();
typedef void (*SetTimeoutCallbackP)(char *);
typedef void (*SetTimeoutCallbackPN)(uint8_t *,int);

struct SetTimeoutTab;

class SetTimeoutClass{
public:
  struct SetTimeoutTab *tbl;
  unsigned short nqu;
  SetTimeoutClass(void);
  void tabshift(SetTimeoutTab *,int blocks,bool insert);
  long set(SetTimeoutCallback,int delay);
  long set(char *message,SetTimeoutCallbackP,int delay);
  long set(uint8_t *message,int length,SetTimeoutCallbackPN,int delay);
  int clear(long);
  int lookup(long);
  SetTimeoutCallback spinOnce(void);
};

extern SetTimeoutClass setTimeout;

#endif
