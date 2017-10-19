//Compile with g++ -Wall -o tourGuide -lAria -ldl `pkg-config --cflags --libs opencv` -lpthread -L /usr/local/Aria/lib -I/usr/local/Aria/include tourGuide.cpp
#include "../include/eyesim.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

/* defines */

#define CAMERA_ON 0

#define DRIVE_SPEED 500

#define REQUIRED_ARGUMENTS 1
#define IMAGE_SCALE_FACTOR 10
#define WINDOW_SIZE 300 // window size of image display
#define SUBTRACT_HISTORY 50 // compared histroy of image in subtractor
#define SUBTRACT_THRESHOLD 3 // threshold value for subtractor
#define COS_30 0.8660254038
#define COS_45 0.7071067812
#define COS_60 0.5
#define COS_75 0.2588190451

/* Namespace */
using namespace std;
using namespace cv;

/* Global Variables */
ArLaser* g_laser;
eyesim robot;
/* Declare Function */

void printUsage();

int SIMLaserScan(double *scan);

class MotionDetector {
public:
    MotionDetector(int history = 500, double varThreshold = 16,
                   bool detectShadows = true) {
        bgSubtractor = createBackgroundSubtractorMOG2(history, varThreshold, detectShadows);
    }

    void imShowForeground(int windowSizeX = 0, int windowSizeY = 0) {

        if ((windowSizeX == 0) & (windowSizeY == 0)) {
            namedWindow("foreground");
        } else {
            namedWindow("foreground", WINDOW_NORMAL);
        }

        resizeWindow("foreground", windowSizeX, windowSizeY);
        imshow("foreground", foreground);
    }

    void imShowBackground(int windowSizeX = 0, int windowSizeY = 0) {

        if ((windowSizeX == 0) & (windowSizeY == 0)) {
            namedWindow("background");
        } else {
            namedWindow("background", WINDOW_NORMAL);
        }

        resizeWindow("background", windowSizeX, windowSizeY);
        imshow("background", background);
    }

    Mat Detect(Mat frame) {
        this->bgSubtractor->apply(frame, this->foreground);
        this->bgSubtractor->getBackgroundImage(this->background);
        erode(this->foreground, this->foreground, Mat());
        dilate(this->foreground, this->foreground, Mat());
        findContours(this->foreground, this->contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        drawContours(frame, this->contours, -1, Scalar(0, 0, 255), 2);
        return frame;
    }


private:
    Ptr<BackgroundSubtractorMOG2> bgSubtractor;
    Mat foreground;
    Mat background;
    vector<vector<Point>> contours;
};

void LeftFollow(double dist, double speed){
    int i;
    double scan[181];
    double frontDist = dist/COS_30;
    SIMLaserScan(scan);
    for(i=45;i<=135;i++){
        if(scan[i]<frontDist){
            robot.comInt(ArCommands::VEL, 0);
            cout<<"avoid front obstacle"<<endl;
            robot.comInt(ArCommands::RVEL, -25);
            return;
        }
    }
    cout<<"front clear"<<endl;
    robot.comInt(ArCommands::VEL, (short)speed);
    double err_45 = (scan[135]*COS_45 - dist)/10;
    if(err_45 < speed/5) {
        robot.comInt(ArCommands::RVEL, (short)err_45);
        return;
    }
    double err_60 = (scan[150]*COS_60 - dist)/10;
    if(err_60<speed/5) {
        robot.comInt(ArCommands::RVEL, (short)err_60);
        return;
    }
    double err_75 = scan[165]*COS_75 - dist;
    if(err_75<speed/5/10) {
        robot.comInt(ArCommands::RVEL, (short)err_75);
        return;
    }
    double err_90 = scan[180] - dist;
    if(err_90>speed/5/10){
        err_90 = speed/5/10;
    }
    robot.comInt(ArCommands::RVEL, (short)err_90);

}

/* Main */

int main(int argc, char *argv[]) {
    //Usage
//    if (argc != REQUIRED_ARGUMENTS) {
//        printUsage();
//        exit(EXIT_FAILURE);
//    }

    Aria::init();
    ArArgumentParser parser(&argc,argv);
    ArSimpleConnector connector(&parser);           //inst connector
    parser.loadDefaultArguments();
    ArRobotConnector robotConnector(&parser, &robot);
    ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

    if (!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "lasersExample: Could not connect to the robot->");
        if (parser.checkHelpAndWarnUnparsed()) {
            // -help not given
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    if (!Aria::parseArgs()) {
        Aria::logOptions();
        Aria::exit(2);
        return 2;
    }

    ArLog::log(ArLog::Normal, "lasersExample: Connected to robot->");
    // Start the robot processing cycle running in the background.
    // True parameter means that if the connection is lost, then the
    // run loop ends.
    robot.runAsync(true);
    // Connect to laser(s) as defined in parameter files.
    // (Some flags are available as arguments to connectLasers() to control error behavior and to control which lasers are put in the list of lasers stored by ArRobot. See docs for details.)
    if (!laserConnector.connectLasers()) {
//        ArLog::log(ArLog::Terse, "Could not connect to configured lasers. Exiting.");
        Aria::exit(3);
        return 3;
    }
    // Allow some time to read laser data
    ArUtil::sleep(500);
    ArLog::log(ArLog::Normal, "Connected to all lasers.");
    // Get a pointer to ArRobot's list of connected lasers. We will lock the robot while using it to prevent changes by tasks in the robot's background task thread or any other threads. Each laser has an index. You can also store the laser's index or name (laser->getName()) and use that to get a reference (pointer) to the laser object using ArRobot::findLaser().
    map<int, ArLaser *> *lasers = robot.getLaserMap();
    map<int, ArLaser *>::const_iterator it = lasers->begin();
    int laserIndex = (*it).first;
    g_laser = (*it).second;
    g_laser->lockDevice();
    // The current readings are a set of obstacle readings (with X,Y positions as well as other attributes) that are the most recent set from teh laser.
    list<ArPoseWithTime *> *currentReadings = g_laser->getCurrentBuffer(); // see ArRangeDevice interface doc
    // There is a utility to find the closest reading wthin a range of degrees around the laser, here we use this laser's full field of view (start to end)
    // If there are no valid closest readings within the given range, dist will be greater than laser->getMaxRange().
    double angle = 0;
    double dist = g_laser->currentReadingPolar(g_laser->getStartDegrees(), g_laser->getEndDegrees(), &angle);
    ArLog::log(ArLog::Normal,
               "g_laser #%d (%s): %s. Have %d 'current' readings. Closest reading is at %3.0f degrees and is %2.4f meters away.",
               laserIndex, g_laser->getName(), (g_laser->isConnected() ? "connected" : "NOT CONNECTED"),
               currentReadings->size(), angle, dist / 1000.0);
    g_laser->unlockDevice();
    robot.enableMotors();
    while(1){
        LeftFollow(750,DRIVE_SPEED);
    }

//    int scan[181];

//    sleep(2);
//    SIMLaserScan(scan);

//    int lineSpeed,angSpeed;
//    int x,y,phi;
//    int i=0;
//    robot.VWSetPosition(0,0,0);
//    robot.SetMaxSpeed(5000,1000);
//    robot.GetMaxSpeed(&lineSpeed,&angSpeed);
//    robot.VWStraight(1000,100);

//    robot.lock();
//    robot.enableMotors();
//    ArUtil::sleep(10);
//    cout<<2<<endl;
//    robot.setVel(250);
//    robot.setRotVel(45);
//    robot.unlock();
//    robot.VWGetSpeed(&lineSpeed,&angSpeed);
//    cout<<" linS: "<<lineSpeed<<" angS: "<<angSpeed<<endl;
//    while(scan[90]>1000){
//        SIMLaserScan(scan);
//    }
//    robot.lock();
//    robot.stop();
//    robot.unlock();
//    robot.VWGetPosition(&x,&y,&phi);
//    cout<<" x:"<<x<<" y:"<<y<<" phi:"<< phi<<endl;

    if(CAMERA_ON) {
        //  OpenCV
        VideoCapture cap;
        if (!cap.open(1)) {
            cout << "Unable to open webcam\n";
            exit(EXIT_FAILURE);
        }
        /* video thread */
        Mat frame;
        MotionDetector motionDetector = MotionDetector(SUBTRACT_HISTORY, SUBTRACT_THRESHOLD);

        cap >> frame;
        Size frameSize(40, 30);
        namedWindow("webcam", WINDOW_NORMAL);
        resizeWindow("webcam", frame.cols, frame.rows);

        while (waitKey(10) != 27) {
            cap >> frame;
            if (frame.empty()) break; // end of video stream
            resize(frame, frame, frameSize);
            frame = motionDetector.Detect(frame);
            imshow("webcam", frame);
        }
    }
    Aria::exit(0);
    return 0;
}

/* Define Functions */

/* Usage Information */
void printUsage() {
    cout << "Usage: " << endl;
    cout << "\ttourGuide ";
    // add arguments here for help information
    cout << "";
    cout << endl;
}


int SIMLaserScan(double* scan) {
    int i;
    int laserIndex;
    int index = 0;
    double dist = 0;
    double angle = 0;
//    for (i = -90; i <= 90; i++) {
        map<int, ArLaser *> *lasers = robot.getLaserMap();
        map<int, ArLaser *>::const_iterator it = lasers->begin();
        g_laser = (*it).second;
        g_laser->lockDevice();
        list<ArPoseWithTime *> *currentReadings = g_laser->getCurrentBuffer(); // see ArRangeDevice interface doc
        // There is a utility to find the closest reading wthin a range of degrees around the laser, here we use this laser's full field of view (start to end)
        // If there are no valid closest readings within the given range, dist will be greater than laser->getMaxRange().
        for (i = -90; i <= 90; i++) {
        dist = g_laser->currentReadingPolar(i - 0.8, i + 0.8, &angle);

        scan[index] = dist;
//        cout<<scan[index]<<endl;
            index++;
    }
    g_laser->unlockDevice();
//    cout<<scan[89]<<endl;
    return 0;
}