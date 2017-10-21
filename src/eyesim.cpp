//
// Created by chaoz on 16/10/17.
//

#include "eyesim.h"

//
int eyesim::VWSetSpeed(int linSpeed, int angSpeed) {
    this->lock();
    if (this->getTransVelMax() < linSpeed) {
//        cout<<"Reach Max TransVel!\n" << endl;
        this->unlock();
        return (1);
    }
    if (this->getRotVelMax() < angSpeed) {
        this->unlock();
//        cout<<"Reach Max RotVel!\n"<< endl;
        return (1);
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
    ArPose thisPose(x, y, phi);
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
    *x = (int) thisPose.getX();
    *y = (int) thisPose.getY();
    *phi = (int) thisPose.getTh();
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
    return this->isMoveDone() && this->isHeadingDone();
}

int eyesim::VWWait(void) {
    while (!this->isMoveDone() || !this->isHeadingDone());
    return (0);
}

bool eyesim::VWStalled(void) {
    if (this->isLeftMotorStalled() || this->isRightMotorStalled()) {
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

// version 1
//void eyesim::LeftFollow(double dist, double speed) {
//    int i;
//    double scan[181];
//    dist += 350;
//    double frontDist = dist;
//    SIMLaserScan(scan);
//    for (i = 45; i <= 135; i++) {
//        if (scan[i] < frontDist) {
//            this->lock();
//            this->comInt(ArCommands::VEL, 0);
//            cout<<"avoid front obstacle"<<endl;
//            this->comInt(ArCommands::RVEL, -25);
//            this->unlock();
//            return;
//        }
//    }
//    cout<<"front clear"<<endl;
//    this->lock();
//    this->comInt(ArCommands::VEL, (short) speed);
//    this->unlock();
//    double err_45 = (scan[135] * COS_45 - dist) / 10;
//    if (abs(err_45) < 10) {
//        this->lock();
//        this->comInt(ArCommands::RVEL, 0);
//        this->unlock();
//        return;
//    }
//    if (err_45 < speed / 5 / 6) {
//        this->lock();
//        this->comInt(ArCommands::RVEL, -(short) err_45);
//        this->unlock();
//        return;
//    }
//    double err_60 = (scan[150] * COS_60 - dist) / 10;
//    if (abs(err_60) < 10) {
//        this->lock();
//        this->comInt(ArCommands::RVEL, 0);
//        this->unlock();
//        return;
//    }
//    if (err_60 < speed / 5 / 6) {
//        this->lock();
//        this->comInt(ArCommands::RVEL, -(short) err_60);
//        this->unlock();
//        return;
//    }
//    double err_75 = (scan[165] * COS_75 - dist) / 10;
//    if (abs(err_75) < 10) {
//        this->lock();
//        this->comInt(ArCommands::RVEL, 0);
//        this->unlock();
//        return;
//    }
//    if (err_75 < speed / 5 / 5) {
//        this->lock();
//        this->comInt(ArCommands::RVEL, -(short) err_75);
//        this->unlock();
//        return;
//    }
//    double err_90 = (scan[180] - dist) / 10;
//    if (abs(err_90) < 10) {
//        this->lock();
//        this->comInt(ArCommands::RVEL, 0);
//        this->unlock();
//        return;
//    }
//    if (err_90 > speed / 5 / 6) {
//        err_90 = speed / 5 / 6;
//    }
//    this->lock();
//    this->comInt(ArCommands::RVEL, -(short) err_90);
//    this->unlock();
//}

//version 2
//void eyesim::LeftFollow(double dist, double speed) {
//    int i;
//    double scan[181];
//    dist += 350;
//    double frontDist = dist;
//    SIMLaserScan(scan);
//    for (i = 45; i <90; i++) {
//        if (scan[i] < frontDist) {
//            this->lock();
//            this->comInt(ArCommands::STOP, 1);
//            cout<<"avoid front obstacle"<<endl;
//            this->comInt(ArCommands::RVEL, 25);
//            this->unlock();
//            return;
//        }
//        if (scan[180-i] < frontDist) {
//            this->lock();
//            this->comInt(ArCommands::STOP, 1);
//            cout<<"avoid front obstacle"<<endl;
//            this->comInt(ArCommands::RVEL, -25);
//            this->unlock();
//            return;
//        }
//    }
//    cout<<"front clear"<<endl;
//    double errLeft;
//    double errRight;
//    for(i = 0;i<45;i++){
//        errLeft = scan[180-i]/cosArray[180-i] - dist;
//        errRight = scan[i]/cosArray[i] - dist;
//        if(errLeft<0&&errRight<0){
//            this->lock();
//            this->comInt(ArCommands::STOP, 1);
//            cout<<"avoid narrow path"<<endl;
//            this->comInt(ArCommands::RVEL, -25);
//            this->unlock();
//            return;
//        }
//        if(errLeft<0) {
//            errLeft = -errLeft;
//            if(errLeft>1000){
//                errLeft = 1000;
//            }
//            errLeft/=500;
//            this->lock();
//            this->comInt(ArCommands::VEL,(short)speed);
//            cout<<"avoid left obstacle"<<endl;
//            this->comInt(ArCommands::RVEL,(short)(errLeft));
//            this->unlock();
//            return;
//        }
//
//        if(errRight<0) {
//            if(errRight<-1000){
//                errLeft =-1000;
//            }
//            errLeft/=500;
//            this->lock();
//            this->comInt(ArCommands::VEL,(short)speed);
//            cout<<"avoid left obstacle"<<endl;
//            this->comInt(ArCommands::RVEL,(short)(errLeft));
//            this->unlock();
//            return;
//        }
//    }
//    this->comInt(ArCommands::VEL,(short)speed);
//}

//version 3
void eyesim::LeftFollow(double dist, double speed) {
    int i;
    double scan[181];
    dist += 350;
    double frontDist = dist;
    SIMLaserScan(scan);
    for (i = 45; i <90; i++) {
        if (scan[i] < frontDist) {
            this->lock();
            this->comInt(ArCommands::STOP, 1);
            cout<<"avoid front obstacle"<<endl;
            this->comInt(ArCommands::RVEL, 25);
            this->unlock();
            return;
        }
        if (scan[180-i] < frontDist) {
            this->lock();
            this->comInt(ArCommands::STOP, 1);
            cout<<"avoid front obstacle"<<endl;
            this->comInt(ArCommands::RVEL, -25);
            this->unlock();
            return;
        }
    }
    cout<<"front clear"<<endl;
    double errLeft;
    double errRight;
    for(i = 0;i<45;i++){
        errLeft = scan[180-i]/cosArray[180-i] - dist;
        errRight = scan[i]/cosArray[i] - dist;
        if(errLeft<0&&errRight<0){
            this->lock();
            this->comInt(ArCommands::STOP, 1);
            cout<<"avoid narrow path"<<endl;
            this->comInt(ArCommands::RVEL, -25);
            this->unlock();
            return;
        }
        if(errLeft<0) {
            errLeft = -errLeft;
            if(errLeft>1000){
                errLeft = 1000;
            }
            errLeft/=100;
            this->lock();
            this->comInt(ArCommands::VEL,(short)speed);
            cout<<"avoid left obstacle"<<endl;
            this->comInt(ArCommands::RVEL,(short)(errLeft));
            this->unlock();
            return;
        }
        if(errRight<0) {
            if(errRight<-1000){
                errLeft =-1000;
            }
            errLeft/=100;
            this->lock();
            this->comInt(ArCommands::VEL,(short)speed);
            cout<<"avoid left obstacle"<<endl;
            this->comInt(ArCommands::RVEL,(short)(errLeft));
            this->unlock();
            return;
        }
    }
    double err = (scan[180] - scan[0])/100;
    if(err>10){
        err = 10;
    }
    if(err<-10) {
        err = -10;
    }
    this->lock();
    this->comInt(ArCommands::VEL,(short)speed);
    this->comInt(ArCommands::RVEL,(short)err);
    this->unlock();
}

void eyesim::SIMLaserScan(double *scan) {
    int i;
    int index = 0;
    double dist = 0;
    double angle = 0;
    this->lock();
    map<int, ArLaser *> *lasers = this->getLaserMap();
    this->unlock();
    map<int, ArLaser *>::const_iterator it = lasers->begin();
    ArLaser *laser = (*it).second;

//    laser->lockDevice();
    if (laser->isConnected()) {
//        cout<<"laser connected"<<endl;
    }
    for (i = -90; i <= 90; i++) {
        dist = laser->currentReadingPolar(i - 0.5, i + 0.49, &angle);

        scan[index] = dist;
        index++;
    }
//    laser->unlockDevice();
}

eyesim::eyesim() {
    for (int i = 0; i < 90; i++) {
        cosArray[i] = cosArray[180 - i] = cos(i * PI / 180);
    }
}

void eyesim::DriveLeftFreeSpace(double speed) {
    double scan[181];

}

ArLaser *eyesim::GetLaser() {
    return (*(this->getLaserMap()->begin())).second;
}
