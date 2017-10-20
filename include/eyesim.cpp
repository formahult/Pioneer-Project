//
// Created by chaoz on 16/10/17.
//

#include "eyesim.h"
//
int eyesim::VWSetSpeed(int linSpeed, int angSpeed) {
    this->lock();
    if(this->getTransVelMax()<linSpeed){
        cout<<"Reach Max TransVel!\n" << endl;
        this->unlock();
        return (1);
    }
    if(this->getRotVelMax()<angSpeed){
        this->unlock();
        cout<<"Reach Max RotVel!\n"<< endl;
        return(1);
    }
    this->setVel(linSpeed);
    this->setRotVel(angSpeed);
    this->unlock();
    return (0);
}
//
int eyesim::VWGetSpeed(int *linSpeed, int *angSpeed) {
    this->lock();
    *angSpeed = (int) this->getRotVel();
    *linSpeed = (int) this->getVel();
    this->unlock();
    return (0);
}
//
int eyesim::VWSetPosition(int x, int y, int phi) {
    ArPose thisPose(x,y,phi);
    this->lock();
    this->moveTo(thisPose);
    this->unlock();
    return (0);
}
//
int eyesim::VWGetPosition(int *x, int *y, int *phi) {
    this->lock();
    ArPose thisPose = this->getPose();
    this->unlock();
    *x = (int)thisPose.getX();
    *y = (int)thisPose.getY();
    *phi = (int)thisPose.getTh();
    return (0);
}
//
int eyesim::VWStraight(int dist, int linSpeed) {
    this->lock();
    this->setVel(linSpeed);
    this->move(dist);
    this->unlock();
    return (0);
}

int eyesim::VWTurn(int angle, int ang_speed) {
    this->lock();
    this->setRotVel(ang_speed);
    this->setHeading(angle);
    this->unlock();
    return (0);
}

int eyesim::VWCurve(int dist, int angle, int lin_speed) {
    double time = dist / lin_speed;
    double rotVel = angle / time;
    this->lock();
    this->setVel(lin_speed);
    this->setRotVel(rotVel);
//    this->move(dist);
//    this->setHeading(angle);
    this->unlock();
    return (0);
}

int eyesim::VWDrive(int dx, int dy, int lin_speed) {
    return (0);
}
//
int eyesim::VWDone(void) {
    return this->isMoveDone()&&this->isHeadingDone();
}

int eyesim::VWWait(void) {
    while(!this->isMoveDone()||!this->isHeadingDone());
    return (0);
}

bool eyesim::VWStalled(void) {
    if(this->isLeftMotorStalled()||this->isRightMotorStalled()){
        return (true);
    }
    return (false);
}

int eyesim::Terminate() {
    Aria::exit(0);
    return 0;
}

int eyesim::GetMaxSpeed(int *linMax, int *angMax) {
    this->lock();
    *linMax = (int) (this->getTransVelMax());
    *angMax = (int) (this->getRotVelMax());
    this->unlock();
    return 0;
}

int eyesim::SetMaxSpeed(int linMax, int angMax) {
    this->lock();
    this->setTransVelMax(linMax);
    this->setRotVelMax(angMax);
    this->unlock();
    return 0;
}

void eyesim::LeftFollow(double dist, double speed) {
    int i;
    double scan[181];
    dist+=350;
    double frontDist = dist;
    SIMLaserScan(scan);
    for(i=45;i<=135;i++){
        if(scan[i]<frontDist){
            this->lock();
            this->comInt(ArCommands::VEL, 0);
            cout<<"avoid front obstacle"<<endl;
            this->comInt(ArCommands::RVEL, -25);
            this->unlock();
            return;
        }
    }
    cout<<"front clear"<<endl;
    this->lock();
    this->comInt(ArCommands::VEL, (short)speed);
    this->unlock();
    double err_45 = (scan[135]*COS_45 - dist)/10;
    if(abs(err_45)<10){
        this->lock();
        this->comInt(ArCommands::RVEL, 0);
        this->unlock();
        return;
    }
    if(err_45 < speed/5/6) {
        this->lock();
        this->comInt(ArCommands::RVEL, -(short)err_45);
        this->unlock();
        return;
    }
    double err_60 = (scan[150]*COS_60 - dist)/10;
    if(abs(err_60)<10){
        this->lock();
        this->comInt(ArCommands::RVEL, 0);
        this->unlock();
        return;
    }
    if(err_60<speed/5/6) {
        this->lock();
        this->comInt(ArCommands::RVEL, -(short)err_60);
        this->unlock();
        return;
    }
    double err_75 = (scan[165]*COS_75 - dist)/10;
    if(abs(err_75)<10){
        this->lock();
        this->comInt(ArCommands::RVEL, 0);
        this->unlock();
        return;
    }
    if(err_75<speed/5/5) {
        this->lock();
        this->comInt(ArCommands::RVEL, -(short)err_75);
        this->unlock();
        return;
    }
    double err_90 = (scan[180] - dist)/10;
    if(abs(err_90)<10){
        this->lock();
        this->comInt(ArCommands::RVEL, 0);
        this->unlock();
        return;
    }
    if(err_90>speed/5/6){
        err_90 = speed/5/6;
    }
    this->lock();
    this->comInt(ArCommands::RVEL, -(short)err_90);
    this->unlock();
}

//void eyesim::LeftFollow(double dist, double speed) {
//    double scan[181];
//    dist += 350;
//    SIMLaserScan(scan);
//    double err;
//    double angDist= dist;
//    for(int i = 30; i<=90;i++){
//        if(((scan[i] - angDist)/ERR_COE)<0){
//            this->lock();
//            this->setVel(0);
//            this->setRotVel(-140);
//            this->unlock();
//            cout<<scan[i]<<endl;
//            return;
//        }
//        if(((scan[180-i] - angDist)/ERR_COE)<0){
//            this->lock();
//            this->setVel(0);
//            this->setRotVel(-10);
//            this->unlock();
//            return;
//        }
//    }
//    double curveDist = dist * PI / 2;
//    for(int j = 135; j<181;j++){
//        if((err = (scan[j] - dist / cosArray[j])/ERR_COE) < 0){
//            cout<<err<<endl;
//            VWCurve((int)curveDist,(int)err,(int)speed);
//            return;
//        }
//    }
//    if(err>180){
//        err = 180;
//    }
//    cout<<scan[180]<<" "<<err<<endl;
//    VWCurve((int)curveDist,(int)err,(int)speed);
//
//
//}

void eyesim::SIMLaserScan(double *scan) {
    int i;
    int index = 0;
    double dist = 0;
    double angle = 0;
    const std::list<ArPoseWithTime *> *readingsList;
    std::list<ArPoseWithTime *>::const_iterator RLit;
    this->lock();
    map<int, ArLaser *> *lasers = this->getLaserMap();
    this->unlock();
    map<int, ArLaser *>::const_iterator it = lasers->begin();
    ArLaser* laser = (*it).second;

//    laser->lockDevice();
    if(laser->isConnected()){
        cout<<"laser connected"<<endl;
    }
    for (i = -90; i <= 90; i++) {
        dist = laser->currentReadingPolar(i - 0.5, i + 0.49, &angle);

        scan[index] = dist;
        index++;
    }
//    laser->unlockDevice();
}

eyesim::eyesim() {
    for(int i = 0; i < 90; i++) {
        cosArray[i]=cosArray[180-i] = cos(i*PI/180);
    }
}

void eyesim::DriveLeftFreeSpace(double speed) {
    double scan[181];

}
