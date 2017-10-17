//
// Created by chaoz on 16/10/17.
//

#include "eyesim.h"
//
int eyesim::VWSetSpeed(int linSpeed, int angSpeed) {
    robot.lock();
    if(robot.getTransVelMax()<linSpeed){
        cout<<"Reach Max TransVel!\n" << endl;
        return (1);
    }
    if(robot.getRotVelMax()<angSpeed){
        cout<<"Reach Max RotVel!\n"<< endl;
        return(1);
    }
    robot.setVel(linSpeed);
    robot.setRotVel(angSpeed);
    return (0);
}
//
int eyesim::VWGetSpeed(int *linSpeed, int *angSpeed) {
    *angSpeed = (int) robot.getRotVel();
    *linSpeed = (int) robot.getVel();
    return (0);
}
//
int eyesim::VWSetPosition(int x, int y, int phi) {
    ArPose thisPose(x,y,phi);
    robot.lock();
    robot.moveTo(thisPose);
    robot.unlock();
    return (0);
}
//
int eyesim::VWGetPosition(int *x, int *y, int *phi) {
    ArPose thisPose = robot.getPose();
    *x = (int)thisPose.getX();
    *y = (int)thisPose.getY();
    *phi = (int)thisPose.getTh();
    return (0);
}
//
int eyesim::VWStraight(int dist, int linSpeed) {
    if(robot.isDirectMotion()) robot.clearDirectMotion();
    robot.setVel(linSpeed);
    robot.move(dist);
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

    return robot.isMoveDone(0);
}

int eyesim::VWWait(void) {

    return (0);
}

int eyesim::VWStalled(void) {
    return (0);
}
//
int eyesim::SIMLaserScan(int *scan) {
    const list<ArSensorReading *> *readingsList;
    list<ArSensorReading *>::const_iterator itera;
    int i = -1;

    readingsList = laser.getRawReadings();
    cout<<"1"<<endl;
    for (itera = readingsList->begin(); itera != readingsList->end(); itera++) {
        i++;
        scan[i] = (*itera)->getRange();
        cout<< scan[i]<<endl;
    }

    return 0;
}

eyesim::eyesim(int *argc, char **argv) {
    Aria::init();
    robot.addRangeDevice(&laser);
    ArArgumentParser parser(argc, argv); //inst argument parser
    ArSimpleConnector connector(&parser);           //inst connector
    parser.loadDefaultArguments();

    if (!connector.parseArgs()) {
        cout << "Unknown settings\n";
        Aria::exit(0);
        exit(EXIT_FAILURE);
    }
    if (!connector.connectRobot(&robot)) {
        cout << "Unable to connect\n";
        Aria::exit(0);
        exit(EXIT_FAILURE);
    }
    robot.runAsync(true);
    laser.runAsync();
    if (!connector.connectLaser(&laser)) {
        cout << "Can't connect to laser\n";
        Aria::exit(0);
        exit(EXIT_FAILURE);
    }
    laser.asyncConnect();
    while(!laser.isConnected());
    robot.lock();
    robot.comInt(ArCommands::ENABLE, 1);
    robot.unlock();
    cout<<"Wait 5s to let the laser connection be establised."<<endl;
    sleep(5);
}

int eyesim::Terminate() {
    Aria::exit(0);
    return 0;
}

int eyesim::GetMaxSpeed(int *linMax, int *angMax) {
    *linMax = (int) (robot.getTransVelMax());
    *angMax = (int) (robot.getRotVelMax());
    return 0;
}

int eyesim::SetMaxSpeed(int linMax, int angMax) {
    robot.lock();
    robot.setTransVelMax(linMax);
    robot.setRotVelMax(angMax);
    robot.unlock();
    return 0;
}
