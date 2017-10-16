//Compile with g++ -Wall -o tourGuide -lAria -ldl `pkg-config --cflags --libs opencv` -lpthread -L /usr/local/Aria/lib -I/usr/local/Aria/include tourGuide.cpp

#include "Aria.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <iostream>

/* defines */
#define REQUIRED_ARGUMENTS 1
#define WINDOW_SIZE 300 // window size of image display
#define SUBTRACT_HISTORY 50 // compared histroy of image in subtractor
#define SUBTRACT_THRESHOLD 16 // threshold value for subtractor
#define ROBOT_ENABLE 0  // set as 1 to turn on g_robot connection
#define LASER_LEFT 180
#define LASER_RIGHT 0
#define LASER_FRONT 90

/* Namespace */
using namespace std;
using namespace cv;

/* Global Variables */
ArRobot g_robot;
ArSick g_laser;

/* Declare Function */

void printUsage();

Ptr<int> GetLaser(Ptr<ArSick> laser);

class MotionDetector {
public:
    MotionDetector(int history = 500, double varThreshold = 16,
                        bool detectShadows = true) {
        bgSubtractor = createBackgroundSubtractorMOG2(history, varThreshold, detectShadows);
    }

    void imShowForeground(int windowSizeX = 0, int windowSizeY = 0) {

        if((windowSizeX==0)&(windowSizeY==0)){
            namedWindow("foreground");
        }else {
            namedWindow("foreground", WINDOW_NORMAL);
        }

        resizeWindow("foreground", windowSizeX, windowSizeY);
        imshow("foreground", foreground);
    }
    void imShowBackground(int windowSizeX = 0, int windowSizeY = 0) {

        if((windowSizeX==0)&(windowSizeY==0)){
            namedWindow("background");
        }else {
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

/* Main */

int main(int argc, char *argv[]) {
    //Usage
    if (argc != REQUIRED_ARGUMENTS) {
        printUsage();
        exit(EXIT_FAILURE);
    }

    if (ROBOT_ENABLE) {
        /* initialisation of lbraries */
        //  Aria
        Aria::init();
        g_robot.addRangeDevice(&g_laser);
        ArArgumentParser parser(&argc, argv); //inst argument parser
        ArSimpleConnector connector(&parser);           //inst connector
        parser.loadDefaultArguments();

        /* Connection to robot */
        if (!connector.parseArgs()) {
            cout << "Unknown settings\n";
            Aria::exit(0);
            exit(EXIT_FAILURE);
        }
        if (!connector.connectRobot(&g_robot)) {
            cout << "Unable to connect\n";
            Aria::exit(0);
            exit(EXIT_FAILURE);
        }
        g_robot.runAsync(true);
        g_laser.runAsync();
        if (!connector.connectLaser(&g_laser)) {
            cout << "Can't connect to g_laser\n";
            Aria::exit(0);
            exit(EXIT_FAILURE);
        }
        g_laser.asyncConnect();

        g_robot.lock();
        g_robot.comInt(ArCommands::ENABLE, 1);
        g_robot.unlock();
    }

    //  OpenCV
    VideoCapture cap;
    if (!cap.open(0)) {
        cout << "Unable to open webcam\n";
        exit(EXIT_FAILURE);
    }
    /* video thread */
    Mat frame;
    MotionDetector motionDetector = MotionDetector(SUBTRACT_HISTORY,SUBTRACT_THRESHOLD);
    namedWindow("webcam", WINDOW_NORMAL);
    resizeWindow("webcam", WINDOW_SIZE, WINDOW_SIZE);

    while (waitKey(10) != 27) {
//      system("play -q sound-impressed.wav");
        cap >> frame;
        if (frame.empty()) break; // end of video stream
        frame = motionDetector.Detect(frame);
        imshow("webcam", frame);
    }


    Aria::exit(0);  //Exit Aria

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

Ptr<int> GetLaser(Ptr<ArSick> laser) {
    const list<ArSensorReading*>* readingsList;
    list<ArSensorReading*>::const_iterator it;


}