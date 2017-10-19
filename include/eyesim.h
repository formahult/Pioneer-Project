//
// Created by chaoz on 16/10/17.
//

#ifndef ROBOT_TOUR_EYESIM_H
#define ROBOT_TOUR_EYESIM_H
#include <Aria.h>
#include <iostream>

#define LASER_LEFT 181
#define LASER_RIGHT 0
#define LASER_FRONT 90

using namespace std;

class eyesim: public ArRobot {
private:
    /* data */
public:
    int Terminate();
//    int SIMLaserScan(int*);
    int VWSetSpeed(int, int);
    int GetMaxSpeed(int *,int *);
    int SetMaxSpeed(int,int);

    int VWGetSpeed(int *, int *);

    int VWSetPosition(int, int, int);

    int VWGetPosition(int *, int *, int *);

    int VWStraight(int, int);

    int VWTurn(int, int);

    int VWCurve(int, int, int);

    int VWDrive(int, int, int);

    int VWRemain(void);

    int VWDone(void);

    int VWWait(void);

    int VWStalled(void);
};

#endif //ROBOT_TOUR_EYESIM_H
