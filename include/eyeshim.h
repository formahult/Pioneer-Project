//Aria Header

#ifndef LIBARIA
#include "Aria.h"
#define LIBARIA
#endif

class XArRobot: public ArRobot{
private:
  /* data */

public:
  int VWSetSpeed(int, int);
  int VWGetSpeed(int*, int*);
  int VWSetPosition(int, int, int);
  int VWGetPosition(int*, int*, int*);
  int VWStraight(int, int);
  int VWTurn(int, int);
  int VWCurve(int, int, int);
  int VWDrive(int, int, int);
  int VWRemain(void);
  int VWDone(void);
  int VWWait(void);
  int VWStalled(void);
};

int XArRobot::VWSetSpeed(int linSpeed, int angSpeed){
  return(0);
}
int XArRobot::VWGetSpeed(int *linSspeed, int *angSpeed){
  return(0);
}
int XArRobot::VWSetPosition(int x, int y, int phi){
  return(0);
}
int XArRobot::VWGetPosition(int *x, int *y, int *phi){
  return(0);
}
int XArRobot::VWStraight(int dist, int lin_speed){
  return(0);
}
int XArRobot::VWTurn(int angle, int ang_speed){
  return(0);
}
int XArRobot::VWCurve(int dist, int angle, int lin_speed){
  return(0);
}
int XArRobot::VWDrive(int dx, int dy, int lin_speed){
  return(0);
}
int XArRobot::VWRemain(void){
  return(0);
}
int XArRobot::VWDone(void){
  return(0);
}
int XArRobot::VWWait(void){
  return(0);
}
int XArRobot::VWStalled(void){
  return(0);
}
