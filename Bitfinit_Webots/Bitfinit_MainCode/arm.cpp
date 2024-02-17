#include "AllHeaders.h"

#include <string>
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>

using namespace std;

namespace arm_methods

{
  double angle;
  void swap_move_1(Motor *motors[6],Robot *robot)
  
  {
    arm_pos(0.12,1,motors);
    reset_arm(motors);
    robot->step(1500);
    swap_pos_1(motors);
    robot->step(2000);
    arm_pos(0.09,1,motors);////////
    robot->step(1000);
  }
  
  void swap_move_4(Motor *motors[6],Robot *robot)
  
  {
    arm_methods::arm_pos(0.12,1,motors);
    robot->step(1000);
             //arm_methods::reset_arm(motors);
    arm_methods::swap_pos_1(motors);
    robot->step(3500);
    arm_methods::arm_pos(0.08,1,motors);
    robot->step(1000);
  }
  void swap_pos_1(Motor *motors[6])
  {
    
             
             
    motors[2]->setVelocity(1.0);
    motors[2]-> setPosition(1.57);
    motors[3]->setVelocity(1.0);
    motors[3]-> setPosition(1.57);
    
    
  }
  void swap_move_2(Motor *motors[6])
  {
    motors[2]->setVelocity(1.0);
    motors[2]-> setPosition(-1.57);
    motors[3]->setVelocity(1.0);
    motors[3]-> setPosition(-1.57);
  }
  void reset_arm(Motor *motors[6])
  {
    motors[2]->setVelocity(1.0);
    motors[2]-> setPosition(0);
    motors[3]->setVelocity(1.0);
    motors[3]-> setPosition(0);
  }
  void completely_close_gripper(Motor *motors[6])
  {
  motors[4]->setVelocity(1.0);
  motors[4]-> setPosition(-0.4);
  motors[5]->setVelocity(1.0);
  motors[5]-> setPosition(0.4);
  }
  void close_gripper(double in,Motor *motors[6])
  {
  motors[4]->setVelocity(0.3);
  motors[4]-> setPosition(in);
  motors[5]->setVelocity(0.3);
  motors[5]-> setPosition(-in);
  }
  void open_gripper(Motor *motors[6])
  {
  motors[4]->setVelocity(1.0);
  motors[4]-> setPosition(1.0472);
  motors[5]->setVelocity(1.0);
  motors[5]-> setPosition(-1.0472);
  }
  void pack_gripper(Motor *motors[6])
  {
  arm_pos(0.1,4,motors);
  completely_close_gripper(motors);
  motors[2]->setVelocity(1.0);
  motors[2]-> setPosition(1.7408);
  motors[3]->setVelocity(1.0);
  motors[3]-> setPosition(1.73);
  }
  void grab_8(Motor *motors[6])
  {
  close_gripper(-0.3,motors);
  }
  void grab_6(Motor *motors[6])
  {
  close_gripper(0.1,motors);
  }
  void grab_4(Motor *motors[6])
  {
  close_gripper(0,motors);
  }
  
  void arm_pos(double val,double speed,Motor *motors[6])
  {
  motors[1]->setVelocity(speed);
  motors[1]-> setPosition(val);
  }
  void position_arm(vector<double> vals,Motor *motors[6],Robot *robot)
  {
  if (vals[0]>8.5)
  {
  motors[2]->setVelocity(7.0);
  motors[2]-> setPosition(1.57);
  motors[3]->setVelocity(7.0);
  motors[3]-> setPosition(-1.57);
  }
  else if (vals[0]<-8.5)
  {
    cout<<"wdfesgrdghdrgde"<<endl;
    motors[2]->setVelocity(7.0);
    motors[2]-> setPosition(-1.57);
    motors[3]->setVelocity(7.0);
    motors[3]-> setPosition(1.57);
  }
  else
  {
    angle = vals[0]/8.5;
    motors[2]->setVelocity(7.0);
    motors[2]-> setPosition(angle);
    motors[3]->setVelocity(7.0);
    motors[3]-> setPosition(-angle);
    
  }
  robot->step(500);
  //motors[1]->setVelocity(1.0);
 // motors[1]-> setPosition(0);
  arm_pos(-0.02,4,motors);
  robot->step(1000);

  }
  
}

