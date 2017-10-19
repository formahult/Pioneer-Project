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
    readingsList = laser->getRawReadings();
    cout<<"1"<<endl;
    for (itera = readingsList->begin(); itera != readingsList->end(); itera++) {
        i++;
        scan[i] = (*itera)->getRange();
        cout<< scan[i]<<endl;
    }

    return 0;
}

eyesim::eyesim(ArArgumentParser aparser) : parser(aparser) {
    Aria::init();
    ArSimpleConnector connector(&parser);           //inst connector
    parser.loadDefaultArguments();

    ArRobotConnector robotConnector(&parser, &robot);
    ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

    if (!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "lasersExample: Could not connect to the robot.");
        if (parser.checkHelpAndWarnUnparsed()) {
            // -help not given
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    if (!Aria::parseArgs()) {
        Aria::logOptions();
        Aria::exit(2);
        return;
    }

    ArLog::log(ArLog::Normal, "lasersExample: Connected to robot.");
    // Start the robot processing cycle running in the background.
    // True parameter means that if the connection is lost, then the
    // run loop ends.
    robot.runAsync(true);
    // Connect to laser(s) as defined in parameter files.
    // (Some flags are available as arguments to connectLasers() to control error behavior and to control which lasers are put in the list of lasers stored by ArRobot. See docs for details.)
    if (!laserConnector.connectLasers()) {
        ArLog::log(ArLog::Terse, "Could not connect to configured lasers. Exiting.");
        Aria::exit(3);
        return;
    }
    // Allow some time to read laser data
    ArUtil::sleep(500);
    ArLog::log(ArLog::Normal, "Connected to all lasers.");

    int numLasers = 0;
    // Get a pointer to ArRobot's list of connected lasers. We will lock the robot while using it to prevent changes by tasks in the robot's background task thread or any other threads. Each laser has an index. You can also store the laser's index or name (laser->getName()) and use that to get a reference (pointer) to the laser object using ArRobot::findLaser().
    robot.lock();
    map<int, ArLaser *> *lasers = robot.getLaserMap();
    map<int, ArLaser *>::const_iterator i = lasers->begin();
    int laserIndex = (*i).first;
    laser = (*i).second;
    laser->lockDevice();
    // The current readings are a set of obstacle readings (with X,Y positions as well as other attributes) that are the most recent set from teh laser.
    list<ArPoseWithTime *> *currentReadings = laser->getCurrentBuffer(); // see ArRangeDevice interface doc
    // There is a utility to find the closest reading wthin a range of degrees around the laser, here we use this laser's full field of view (start to end)
    // If there are no valid closest readings within the given range, dist will be greater than laser->getMaxRange().
    double angle = 0;
    double dist = laser->currentReadingPolar(laser->getStartDegrees(), laser->getEndDegrees(), &angle);
    ArLog::log(ArLog::Normal,
               "Laser #%d (%s): %s. Have %d 'current' readings. Closest reading is at %3.0f degrees and is %2.4f meters away.",
               laserIndex, laser->getName(), (laser->isConnected() ? "connected" : "NOT CONNECTED"),
               currentReadings->size(), angle, dist / 1000.0);
    laser->unlockDevice();
    // Unlock robot and sleep for 5 seconds before next loop.
    robot.unlock();
    ArUtil::sleep(5000);

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
