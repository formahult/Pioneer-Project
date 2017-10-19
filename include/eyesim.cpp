//
// Created by chaoz on 16/10/17.
//

#include "eyesim.h"
//
int eyesim::VWSetSpeed(int linSpeed, int angSpeed) {
    if(this->getTransVelMax()<linSpeed){
        cout<<"Reach Max TransVel!\n" << endl;
        return (1);
    }
    if(this->getRotVelMax()<angSpeed){
        cout<<"Reach Max RotVel!\n"<< endl;
        return(1);
    }
    this->setVel(linSpeed);
    this->setRotVel(angSpeed);
    return (0);
}
//
int eyesim::VWGetSpeed(int *linSpeed, int *angSpeed) {
    *angSpeed = (int) this->getRotVel();
    *linSpeed = (int) this->getVel();
    return (0);
}
//
int eyesim::VWSetPosition(int x, int y, int phi) {
    ArPose thisPose(x,y,phi);
    this->moveTo(thisPose);
    return (0);
}
//
int eyesim::VWGetPosition(int *x, int *y, int *phi) {
    ArPose thisPose = this->getPose();
    *x = (int)thisPose.getX();
    *y = (int)thisPose.getY();
    *phi = (int)thisPose.getTh();
    return (0);
}
//
int eyesim::VWStraight(int dist, int linSpeed) {
    if(this->isDirectMotion()) this->clearDirectMotion();
    this->move(dist);
    return (0);
}

int eyesim::VWTurn(int angle, int ang_speed) {
    return (0);
}

int eyesim::VWCurve(int dist, int angle, int lin_speed) {
    return (0);
}

int eyesim::VWDrive(int dx, int dy, int lin_speed) {
    return (0);
}

int eyesim::VWRemain(void) {
    return (0);
}
//
int eyesim::VWDone(void) {

    return this->isMoveDone(0);
}

int eyesim::VWWait(void) {

    return (0);
}

int eyesim::VWStalled(void) {
    return (0);
}

int eyesim::Terminate() {
    Aria::exit(0);
    return 0;
}

int eyesim::GetMaxSpeed(int *linMax, int *angMax) {
    *linMax = (int) (this->getTransVelMax());
    *angMax = (int) (this->getRotVelMax());
    return 0;
}

int eyesim::SetMaxSpeed(int linMax, int angMax) {
    this->setTransVelMax(linMax);
    this->setRotVelMax(angMax);
    return 0;
}
