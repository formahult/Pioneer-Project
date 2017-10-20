//Screen.h
#ifndef LIBCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#define LIBCV
#endif

#include <iostream>

class screen{
  private:
    Mat image;
  public:
    int displayImage(string);
    int displayCoordinate(int x, int y);
}
