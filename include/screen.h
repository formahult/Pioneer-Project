//Screen.h
#ifndef LIBCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#define LIBCV
#endif

#include <iostream>
#include <string>
#include "eyesim.h"

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
