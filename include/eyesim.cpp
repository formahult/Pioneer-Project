//
// Created by chaoz on 16/10/17.
//

#include "eyesim.h"
//
int eyesim::VWSetSpeed(int linSpeed, int angSpeed) {
    this->robot.lock();
    if(this->robot.getTransVelMax()<linSpeed){
        cout<<"Reach Max TransVel!\n" << endl;
        return (1);
    }
    if(this->robot.getRotVelMax()<angSpeed){
        cout<<"Reach Max RotVel!\n"<< endl;
        return(1);
    }
    this->robot.setVel(linSpeed);
    this->robot.setRotVel(angSpeed);
    return (0);
}
//
int eyesim::VWGetSpeed(int *linSpeed, int *angSpeed) {
    *angSpeed = (int) this->robot.getRotVel();
    *linSpeed = (int) this->robot.getVel();
    return (0);
}
//
int eyesim::VWSetPosition(int x, int y, int phi) {
    ArPose thisPose(x,y,phi);
    this->robot.lock();
    this->robot.moveTo(thisPose);
    this->robot.unlock();
    return (0);
}
//
int eyesim::VWGetPosition(int *x, int *y, int *phi) {
    ArPose thisPose = this->robot.getPose();
    *x = (int)thisPose.getX();
    *y = (int)thisPose.getY();
    *phi = (int)thisPose.getTh();
    return (0);
}
//
int eyesim::VWStraight(int dist, int linSpeed) {
    if(this->robot.isDirectMotion()) this->robot.clearDirectMotion();
    this->robot.setVel(linSpeed);
    this->robot.move(dist);
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

int eyesim::VWDone(void) {
    return (0);
}

int eyesim::VWWait(void) {
    return (0);
}

int eyesim::VWStalled(void) {
    return (0);
}

int eyesim::SIMLaserScan(int *scan) {
    const list<ArSensorReading *> *readingsList;
    list<ArSensorReading *>::const_iterator it;
    int i = -1;
    readingsList = this->laser.getRawReadings();
    for (it = readingsList->begin(); it != readingsList->end(); it++) {
        i++;
        scan[i] = (*it)->getRange();
    }
    return 0;
}

eyesim::eyesim(int *argc, char **argv) {
    Aria::init();
    this->robot.addRangeDevice(&this->laser);
    ArArgumentParser parser(argc, argv); //inst argument parser
    ArSimpleConnector connector(&parser);           //inst connector
    parser.loadDefaultArguments();

    if (!connector.parseArgs()) {
        cout << "Unknown settings\n";
        Aria::exit(0);
        exit(EXIT_FAILURE);
    }
    if (!connector.connectRobot(&this->robot)) {
        cout << "Unable to connect\n";
        Aria::exit(0);
        exit(EXIT_FAILURE);
    }
    this->robot.runAsync(true);
    this->laser.runAsync();
    if (!connector.connectLaser(&this->laser)) {
        cout << "Can't connect to this->laser\n";
        Aria::exit(0);
        exit(EXIT_FAILURE);
    }
    this->laser.asyncConnect();

    this->robot.lock();
    this->robot.comInt(ArCommands::ENABLE, 1);
    this->robot.unlock();

}

