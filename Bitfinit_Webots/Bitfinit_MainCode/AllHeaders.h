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
void line_follow(int previous_error);
void turn_right();
void turn_left();
vector<string> check_box(vector<double> vals);
string get_color();
vector<string> if_picked_check(vector<string> content);
void location_update();
void findShortestPath(int s0, int s1, int t0, int t1,int count);
bool check_if_in(int x, int y);
void numgame(int x);
bool nodeExist(int a, int b);
/////////////////////////////////////////////////////////////
vector<int> get_other_robot_loc();
void set_my_loc(vector<int>loc);
void set_my_target(vector<int>loc);
vector<int> get_other_robot_target();
void add_path_nodes(vector<string>data);
vector<vector<int>> get_added_nodes();
////////////////////////////////////////////////////////////////////
//void pick_up_box(vector<double> vals,int type);
//vector<string> check_for_junction();
namespace encoders
{
  void moveLeft(int speed);
  void moveRight(int speed);
  void moveFwd(int speed);
  void moveBwd(int speed);
  void go(double x,double speed, PositionSensor *ps_right,PositionSensor *ps_left,Robot *robot,int timeStep);
  double reset(int speed,PositionSensor *ps_right,PositionSensor *ps_left,Robot *robot,int timeStep, double baseVal_ren,double baseVal_len);
  double roundTo(double number, int n);
  void setLeftSpeed(double speed,Motor *wheels[2]);
  void setRightSpeed(double speed,Motor *wheels[2]);
  void turnAng(double angle,PositionSensor *ps_left,PositionSensor *ps_right,Motor *wheels[2],Robot *robot,int timeStep);
  
  
}
namespace lidar_methods
{
  vector<vector<double>> scan_for_objects(DistanceSensor *lidar,Motor *motors[6],Robot *robot,vector<int> current_node,int dir);
  vector<double> look_for_box(DistanceSensor *lidar,Motor *motors[6],Robot *robot);
  void set_lidar_pos(double val,double vel,Motor *motors[6]);
  bool check_road(DistanceSensor *lidar,Motor *motors[6],Robot *robot);
}

namespace arm_methods
{
  //double angle;
  void completely_close_gripper(Motor *motors[6]);
  void close_gripper(double in,Motor *motors[6]);
  void open_gripper(Motor *motors[6]);
  void pack_gripper(Motor *motors[6]);
  void grab_8(Motor *motors[6]);
  void grab_6(Motor *motors[6]);
  void grab_4(Motor *motors[6]);
  void position_arm(vector<double> vals,Motor *motors[6],Robot *robot);
  void arm_pos(double val,double speed,Motor *motors[6]);
  void reset_arm(Motor *motors[6]);
  void swap_move_1(Motor *motors[6],Robot *robot);
  void swap_move_2(Motor *motors[6]);
  void swap_pos_1(Motor *motors[6]);
  void swap_move_4(Motor *motors[6],Robot *robot);
}

#endif 

