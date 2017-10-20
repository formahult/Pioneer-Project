//
// Created by chaoz on 16/10/17.
//

#ifndef ROBOT_TOUR_EYESIM_H
#define ROBOT_TOUR_EYESIM_H
#include <Aria.h>
#include <iostream>

#define PI 3.14159265
#define COS_30 0.8660254038
#define COS_45 0.7071067812
#define COS_60 0.5
#define COS_75 0.2588190451
#define ERR_COE 5
#define ROBOT_LENGTH_HALF 330
#define ROBOT_WIDTH_HALF 260

using namespace std;

class eyesim: public ArRobot {
private:
    /* data */
    double cosArray[181];
public:
    eyesim();
    int Terminate();
    void SIMLaserScan(double*);
    int VWSetSpeed(int, int);
    int GetMaxSpeed(int *,int *);
    int SetMaxSpeed(int,int);
    ArLaser* GetLaser();
    int VWGetSpeed(int *, int *);

    int VWSetPosition(int, int, int);

    int VWGetPosition(int *, int *, int *);

    int VWStraight(int, int);

    int VWTurn(int, int);

    int VWCurve(int, int, int);

    int VWDrive(int, int, int);

    int VWDone(void);

    int VWWait(void);

    bool VWStalled(void);

    void LeftFollow(double, double);

    void DriveLeftFreeSpace(double);
};
#endif //ROBOT_TOUR_EYESIM_H
