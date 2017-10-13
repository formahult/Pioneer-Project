//Compile with g++ -Wall -o test -lAria -ldl `pkg-config --cflags --libs opencv` -lpthread -L /usr/local/Aria/lib -I/usr/local/Aria/include test.cpp

/* includes */
#ifndef LIBARIA
#include "Aria.h"
#define LIBARIA
#endif
#ifndef LIBCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>
#define LIBCV
#endif
#include <iostream>
#include <cstdlib>

/* defines */
#define REQUIRED_ARGUMENTS 1

using namespace std;
using namespace cv;

/* Usage Information */
void printUsage(){
  cout << "Usage: " << endl;
  cout << "\ttourGuide ";
  // add arguments here for help information
  cout << "";
  cout << endl;
}

/*function prototypes */

/* Main */

int main(int argc, char* argv[]) {
  //Usage
  if(argc != REQUIRED_ARGUMENTS){
    printUsage();
    exit(EXIT_FAILURE);
  }
  /* initialisation of lbraries */
  //  Aria
  Aria::init();
  ArRobot robot;
  ArSick laser;
  robot.addRangeDevice(&laser);
  ArArgumentParser parser(&argc, argv); //inst argument parser
  ArSimpleConnector connector(&parser);           //inst connector
  parser.loadDefaultArguments();
  //  OpenCV
  VideoCapture cap1, cap2;
  if(!cap1.open(1)){
    cout << "Unable to open webcam\n";
    exit(EXIT_FAILURE);
  }
  if(!cap2.open(0)){
    cout << "Unable to open webcam\n";
    exit(EXIT_FAILURE);
  }
  /* video thread */
  bool movement =  false;
  Mat frame1;
  Mat frame2;
  Mat fgMask;
  Ptr<BackgroundSubtractor> pMOG2;



  while(!movement){
      system("play -q sound-impressed.wav");
    cap1 >> frame1;
    cap2 >> frame2;
    if( frame1.empty() ) break; // end of video stream
    circle(frame1, Point(300,300), 50, Scalar(0,255,0),-1);
    imshow("webcam1", frame1);
    imshow("webcam2", frame2);
    if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC
  }
  /* Connection to robot */
  if(!connector.parseArgs()){
    cout << "Unknown settings\n";
    Aria::exit(0);
    exit(EXIT_FAILURE);
  }
  if(!connector.connectRobot(&robot)){
    cout << "Unable to connect\n";
    Aria::exit(0);
    exit(EXIT_FAILURE);
  }
  robot.runAsync(true);
  laser.runAsync();
  if(!connector.connectLaser(&laser)){
    cout << "Can't connect to laser\n";
    Aria::exit(0);
    exit(EXIT_FAILURE);
  }
  laser.asyncConnect();


  robot.lock();
  robot.comInt(ArCommands::ENABLE, 1);
  robot.unlock();

  Aria::exit(0);  //Exit Aria

  return 0;
}
