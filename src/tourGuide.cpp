//Compile with g++ -Wall -o tourGuide -lAria -ldl `pkg-config --cflags --libs opencv` -lpthread -L /usr/local/Aria/lib -I/usr/local/Aria/include tourGuide.cpp

#include "screen.h"
/* defines */

#define CAMERA_ON 0

#define DRIVE_SPEED 500

#define REQUIRED_ARGUMENTS 1
#define IMAGE_SCALE_FACTOR 10
#define WINDOW_SIZE 300 // window size of image display
#define SUBTRACT_HISTORY 50 // compared histroy of image in subtractor
#define SUBTRACT_THRESHOLD 5 // threshold value for subtractor



/* Namespace */
using namespace std;
using namespace cv;

/* Global Variables */
eyesim robot;
/* Declare Function */

void printUsage();

int SIMLaserScan(double *scan);

void printPose(){
    static int i=0;
    if(i==0){
        ArPose thisPose(0.0,0.0,0.0);
        robot.lock();
        robot.moveTo(thisPose);
        robot.unlock();
    }
    cout<<" waypoint:"<<i++<<" x:"<<robot.getX()<<" y:"<<robot.getY()<<" th:"<<robot.getTh()<<endl;
}

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

/* Main */

int main(int argc, char *argv[]) {

    Aria::init();
    ArArgumentParser parser(&argc, argv);
    ArSimpleConnector connector(&parser);
    parser.loadDefaultArguments();
    ArRobotConnector robotConnector(&parser, &robot);
    ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
    if (!robotConnector.connectRobot()) {
        if (parser.checkHelpAndWarnUnparsed()) {
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    if (!Aria::parseArgs()) {
        Aria::logOptions();
        Aria::exit(2);
        exit(2);
    }
    ArKeyHandler *keyHandler = Aria::getKeyHandler();
    if (keyHandler == NULL)
    {
        keyHandler = new ArKeyHandler;
        Aria::setKeyHandler(keyHandler);
        robot.attachKeyHandler(keyHandler);
    }
    cout<<"Press P to print Pose"<<endl;
    ArGlobalFunctor printPoseCB(&printPose);
    keyHandler->addKeyHandler('p', &printPoseCB);
    keyHandler->addKeyHandler('P', &printPoseCB);
    robot.runAsync(true);
    if (!laserConnector.connectLasers()) {
        Aria::exit(3);
        exit(3);
    }
//    ArUtil::sleep(5000);
    robot.enableMotors();
    robot.comInt(ArCommands::ESTALL,0);
    cout << "All connections are done." << endl;
    cout << "Motors are enable." << endl;

    Screen2 screen2(&robot);

    while(waitKey(10)!=27){
        robot.LeftFollow(200,DRIVE_SPEED);
        screen2.UpdateSurrounding(&robot);
        screen2.DisplayImage();
    }

//        Screen screen;
//        screen.displayImage("img/map.png", true);
  //  while(1){
  //               screen.displayRobotLocation(&robot);
  //      robot.LeftFollow(100,DRIVE_SPEED);
  //  }

    // screen.displayImage("img/map.png", true);
//    screen.displayCoordinate(0,0);
//    screen.displayCoordinate(1059,-5321);
//    screen.displayCoordinate(6152,-5574);
//    screen.displayCoordinate(11016,-6102);
//    screen.displayCoordinate(17013,-7130);
//    screen.displayCoordinate(19344,-7729);
//    screen.displayCoordinate(22481,-8701);
//    screen.displayCoordinate(25960,-9890);
//    screen.displayCoordinate(25444,-11541);
//    screen.displayCoordinate(23451,-14043);
    // screen.displayRobotLocation(&robot);

    if(CAMERA_ON) {
        //  OpenCV
        VideoCapture cap;
        if (!cap.open(0)) {
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
