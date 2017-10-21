//screen.cpp
#include "screen.h"


Screen::Screen() {
    image = Mat::zeros(1920, 1080, CV_8UC3);
    xscale = -0.01;
    yscale = -0.01;
    return;
}

int Screen::displayImage(string fileName, bool fullscreen) {
    image = imread(fileName);
    if (!image.data) {
        cout << "could not open" << fileName << endl;
        return (1);
    }

    namedWindow(window, WINDOW_OPENGL);
    if (fullscreen) {
        resizeWindow(window, 1920, 1080);
    }
    imshow(window, image);
    waitKey(10);
    return 0;
}

int Screen::displayCoordinate(int x, int y) {
    Point center = Point(floor(x * xscale) + 713, floor(y * yscale) + 556);
    Scalar color = Scalar(0, 0, 255);
    circle(image, center, 2, color, -1);
    imshow(window, image);
    waitKey(0);
    return 0;
}

int Screen::displayRobotLocation(eyesim *robot) {
    Point location = Point(robot->getY() * xscale, robot->getX());
    Scalar color = Scalar(0, 255, 0);
    circle(image, location, 2, color, -1);
    return 0;
}


Screen2::Screen2(eyesim *robot) {
    colorBlack.val[COLOR_BLUE] = 0;
    colorBlack.val[COLOR_GREEN] = 0;
    colorBlack.val[COLOR_RED] = 0;
    colorOrange.val[COLOR_BLUE] = 0;
    colorOrange.val[COLOR_GREEN] = 140;
    colorOrange.val[COLOR_RED] = 255;
    colorRed.val[COLOR_BLUE] = 0;
    colorRed.val[COLOR_GREEN] = 0;
    colorRed.val[COLOR_RED] = 255;
    colorBlue.val[COLOR_BLUE] = 255;
    colorBlue.val[COLOR_GREEN] = 0;
    colorBlue.val[COLOR_RED] = 0;
    int maxLaserRange = robot->GetLaser()->getMaxRange();
    int scaleFactor = 35;
    MyScaleX = MyScaleY = scaleFactor;
    MyFrontLength = (int) (robot->getRobotLengthFront() / MyScaleY);
    MyHalfWidth = (int) (robot->getRobotWidth() / MyScaleX / 2);
    MyRobotPosition = Point(maxLaserRange / MyScaleX, maxLaserRange / MyScaleY);
    MyLaserPosition = MyRobotPosition;
    MyLaserPosition.y -= MyLaserOffset / MyScaleY;
    MyBackground = Mat(maxLaserRange / MyScaleY, 2 * maxLaserRange / MyScaleX, CV_8UC3, Scalar(255, 255, 255));
    rectangle(MyBackground, Point(MyRobotPosition.x - MyHalfWidth, MyRobotPosition.y),
              Point(MyRobotPosition.x + MyHalfWidth, MyRobotPosition.y - MyFrontLength), Scalar(0, 200, 200), FILLED);
    circle(MyBackground, MyRobotPosition, MyRobotPosition.x, Scalar(0, 255, 0), 1);
    putText(MyBackground,"Max Laser Range",Point(2,10),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));
    putText(MyBackground,"FreeSpace",Point(2,20),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));
    putText(MyBackground,"Max Laser Range",Point(2,30),FONT_HERSHEY_PLAIN,1,Scalar(0,0,0));

    MyBackground.copyTo(MyImage);
    MySize.width = 2 * maxLaserRange / MyScaleX;
    MySize.height = maxLaserRange / MyScaleY;
    namedWindow(MyWindowName, WINDOW_GUI_NORMAL | WINDOW_AUTOSIZE);
}

void Screen2::DisplayImage() {
    imshow(MyWindowName, MyImage);
    waitKey(10);
    MyImage = MyBackground.clone();
}

void Screen2::DisplayBackground() {
    imshow(MyWindowName, MyBackground);
    waitKey(10);
}

void Screen2::UpdateSurrounding(eyesim *robot) {
    double scan[181];

    robot->SIMLaserScan(scan);
    Point obstacle[181];
    double radToDegree = RAD_TO_DEGREE;
    double rad;
    for (int i = 0; i < 181; i++) {
        rad = -i * radToDegree;
        if (scan[i] > robot->GetLaser()->getMaxRange()-100) {
            scan[i] = robot->GetLaser()->getMaxRange()-100;
        }
        obstacle[i].x = (int) (scan[i] * cos(rad) / MyScaleX + MyLaserPosition.x);
        obstacle[i].y = (int) (scan[i] * sin(rad) / MyScaleY + MyLaserPosition.y);
        if (scan[i] > 2000) {
            line(MyImage, MyLaserPosition, obstacle[i], Scalar(0, 255, 255));
        }
        if(scan[i] < 450) {
            line(MyImage, MyLaserPosition, obstacle[i], Scalar(0,0,255));
        }
        MyImage.at<Vec3b>(obstacle[i]) = colorBlack;
    }
    double angleStep = robot->getRotVel() * TIME_STEP*DEGREE_TO_RAD;
    double lengthStep = robot->getVel()* TIME_STEP;
    Point estimatePath;
    double angle,length;
    for(int i = 1; i< 100;i++){
        angle = angleStep*i;
        length = lengthStep * i;
        estimatePath = Point(static_cast<int>(sin(-angle) * length/MyScaleX + MyRobotPosition.x),
                             static_cast<int>(-cos(angle) * length/MyScaleY + MyRobotPosition.y));
        if(estimatePath.y>MyRobotPosition.y) break;
        MyImage.at<Vec3b>(estimatePath) = colorBlue;
    }
//    rectangle(MyImage, Point(MyRobotPosition.x - MyHalfWidth, MyRobotPosition.y),
//              Point(MyRobotPosition.x + MyHalfWidth, MyRobotPosition.y - MyFrontLength), Scalar(0, 200, 200), FILLED);


}
