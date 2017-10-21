//Screen.h
#ifndef SCREEN_H_
#define SCREEN_H_


#include "eyesim.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <iostream>
#include <string>

#define RAD_TO_DEGREE (PI/180)
#define DEGREE_TO_RAD (180/PI)
#define COLOR_RED 2
#define COLOR_GREEN 1
#define COLOR_BLUE 0
#define TIME_STEP 0.05

using namespace cv;
using namespace std;

class Screen{
  private:
    Mat image;
    string window = "Map Window";
    double xscale;
    double yscale;
  public:
    int displayImage(string, bool fullscreen=false);
    int displayCoordinate(int x, int y);
    int displayRobotLocation(eyesim* robot);
    Screen();
};

class Screen2{
private:
    Mat MyImage;
    Mat MyBackground;
    Size MySize;
    string MyWindowName = "robot view";
    int MyScaleX;
    int MyScaleY;
    double MyLaserOffset = 125;
    Point MyRobotPosition;
    Point MyLaserPosition;
    double MyLaserMaxRange;
    Vec3b colorBlack;
    Vec3b colorOrange;
    Vec3b colorRed;
    Vec3b colorBlue;
    Vec3b colorGreen;
    int MyFrontLength;
    int MyHalfWidth;
public:
    explicit Screen2(eyesim*);
    void DisplayImage();
    void DisplayBackground();
    void UpdateSurrounding(eyesim*);
    int SearchFreeSpace(double* scan, double distThres, int countThres);
};


#endif