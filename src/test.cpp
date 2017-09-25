//Compile with g++ -Wall -o test -lAria -ldl -lpthread -L /usr/local/Aria/lib -I/usr/local/Aria/include test.cpp

/* include files */
#ifndef LIBARIA
#include "Aria.h"
#define LIBARIA
#endif

#include "eyeshim.h"
#include <iostream>
#include <cstdlib>

using namespace std;

int main(int argc, char* argv[]) {
  Aria::init();

  XArRobot robot;                       //inst robot with shim extensions
  ArSick laser;
  robot.addRangeDevice(&laser);

  ArArgumentParser parser(&argc, argv); //inst argument parser
  ArSimpleConnector connector(&parser);           //inst connector

  /* Connection to robot */
  parser.loadDefaultArguments();

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
