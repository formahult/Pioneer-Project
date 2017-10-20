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
public:
    explicit Screen2(eyesim*);
    void DisplayImage();
    void DisplayBackground();
    void UpdateSurrounding(eyesim*);
};


#endif