#ifndef ALLHEADERS
#define ALLHEADERS
#include <string>
#include <vector>
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#define ps_vals std::cout << "right val:" << ps_right->getValue()<< ", left val:" << ps_left->getValue() << std::endl
using namespace std;
using namespace webots;

void forward(double ls,double rs);
void stop();
string color_box();
vector <vector <vector <vector <int>>>> image_spliting(const unsigned char *image);
void go_on_line(int baseSpeed,vector<vector <vector <int>>> line,int previous_error);
int box_finding();
void turning(double baseSpeed_change,int previous_error);
void linefollowing();
void turn_right();
void turn_left();


namespace limalka
{
  void moveLeft(int speed);
  void moveRight(int speed);
  void moveFwd(int speed);
  void moveBwd(int speed);
  void go(double x,double speed, PositionSensor *ps_right,PositionSensor *ps_left,Robot *robot,int timeStep);
  double reset(int speed,PositionSensor *ps_right,PositionSensor *ps_left,Robot *robot,int timeStep, double baseVal_ren,double baseVal_len);

}

namespace dumindu
{
  void turnAng(double angle,PositionSensor *ps_left,PositionSensor *ps_right,Motor *wheels[2],Robot *robot);
  double roundTo(double number, int n);
  void setLeftSpeed(double speed,Motor *wheels[2]);
  void setRightSpeed(double speed,Motor *wheels[2]);
  void wallFollow(DistanceSensor *ds[3],Robot *robot,double baseSpeed);
  void wallTurn(DistanceSensor *ds[3]);
}

namespace sithuruwan
{
  int optional_task(DistanceSensor *ds[3],Robot *robot,Motor *motors[5]);
  void lift_ds_panel(Robot *robot,Motor *motors[5]);
  void reset_ds_panel(Robot *robot,Motor *motors[5]);
  int function2(int down,int up);
  int Val_of_N(vector<int> distances);
  vector<int> Proper_factors(int x);
  
  
}
#endif 

