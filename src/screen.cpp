//screen.cpp
#include "screen.h"
#include "eyesim.h"

Screen::Screen(){
  image = Mat::zeros(192,1080,CV_8UC3);
  xscale = -0.01;
  yscale = -0.01;
  return;
}

int Screen::displayImage(string fileName, bool fullscreen){
  image = imread(fileName);
  if(!image.data){
    cout << "could not open" << fileName << endl;
    return(1);
  }

  namedWindow(window, WINDOW_OPENGL);
  if(fullscreen){
    resizeWindow(window, 1920, 1080);
  }
  imshow(window, image);
    waitKey(10);
  return 0;
}

int Screen::displayCoordinate(int x, int y){
  Point center = Point(floor(x*xscale)+713,floor(y*yscale)+556);
  Scalar color = Scalar(0,0,255);
  circle(image, center, 2, color, -1);
  imshow(window, image);
  waitKey(0);
  return 0;
}

int Screen::displayRobotLocation(eyesim* robot){
  Point location = Point(robot->getY()*xscale, robot->getX());
  Scalar color = Scalar(0,255,0);
  circle(image, location, 2, color, -1);
  return 0;
}
