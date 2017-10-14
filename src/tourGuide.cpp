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
#define ROBOT_ENABLE 0  // set as 1 to turn on robot connection

using namespace std;
using namespace cv;


/* Usage Information */
void printUsage() {
    cout << "Usage: " << endl;
    cout << "\tourGuide ";
    // add arguments here for help information
    cout << "";
    cout << endl;
}

Mat DetectMotion(Mat frame, Ptr<BackgroundSubtractorMOG2> subtractor) {

    Mat back;
    Mat fore;

    namedWindow("Background", WINDOW_NORMAL);
    resizeWindow("Background", WINDOW_SIZE, WINDOW_SIZE);
    namedWindow("foreground", WINDOW_NORMAL);
    resizeWindow("foreground", WINDOW_SIZE, WINDOW_SIZE);

    vector<vector<Point>> contours;

    subtractor->apply(frame, fore);
    subtractor->getBackgroundImage(back);
    erode(fore, fore, Mat());
    dilate(fore, fore, Mat());
    findContours(fore, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    drawContours(frame, contours, -1, Scalar(0, 0, 255), 2);
    imshow("Background", back);
    imshow("foreground", fore);
    return frame;
}

/*function prototypes */

/* Main */

int main(int argc, char *argv[]) {
    //Usage
    if (argc != REQUIRED_ARGUMENTS) {
        printUsage();
        exit(EXIT_FAILURE);
    }
    /* initialisation of lbraries */
    if (ROBOT_ENABLE) {
        //  Aria
        Aria::init();
        ArRobot robot;
        ArSick laser;
        robot.addRangeDevice(&laser);
        ArArgumentParser parser(&argc, argv); //inst argument parser
        ArSimpleConnector connector(&parser);           //inst connector
        parser.loadDefaultArguments();

        /* Connection to robot */
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

        robot.lock();
        robot.comInt(ArCommands::ENABLE, 1);
        robot.unlock();
    }

    //  OpenCV
    VideoCapture cap1;
    if (!cap1.open(0)) {
        cout << "Unable to open webcam1\n";
        exit(EXIT_FAILURE);
    }
    /* video thread */
    Mat frame1;

    Ptr<BackgroundSubtractorMOG2> subtractor = createBackgroundSubtractorMOG2(SUBTRACT_HISTORY, SUBTRACT_THRESHOLD,
                                                                              false);
    namedWindow("webcam1", WINDOW_NORMAL);
    resizeWindow("webcam1", WINDOW_SIZE, WINDOW_SIZE);

    while (waitKey(10) != 27) {
//      system("play -q sound-impressed.wav");
        cap1 >> frame1;
        if (frame1.empty()) break; // end of video stream

        frame1 = DetectMotion(frame1, subtractor);

        imshow("webcam1", frame1);
    }


    Aria::exit(0);  //Exit Aria

    return 0;
}
