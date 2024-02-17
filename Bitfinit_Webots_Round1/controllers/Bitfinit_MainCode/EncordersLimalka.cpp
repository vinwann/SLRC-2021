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

namespace limalka
{
  double reset(int speed,PositionSensor *ps_right,PositionSensor *ps_left,Robot *robot,int timeStep, double baseVal_ren,double baseVal_len)
  {
    double ps_r = ps_right -> getValue() - baseVal_ren;
    double ps_l = ps_left -> getValue() - baseVal_len;
    double diff;
    double ret_val = ps_l - ps_r;
    if (ps_r == ps_l) return ret_val;
    if (ps_r > ps_l)
    {
      moveRight(speed);
      while (robot->step(timeStep) != -1) 
      {
        ps_r = ps_right -> getValue() - baseVal_ren;
        ps_l = ps_left -> getValue() -  baseVal_len;
        diff = ps_r - ps_l;
        //std::cout << diff << std::endl;
        //ps_vals;
        //robot->step(1);
        if (diff <= -0.6)
        {
          stop();
          //std::cout << "stoped....!!!" << std::endl;
          //ps_vals;
          break;
        }
      }
    }
    else
    {
      moveLeft(speed);
      while (robot->step(timeStep) != -1) 
      {
        ps_r = ps_right -> getValue() - baseVal_ren;
        ps_l = ps_left -> getValue() - baseVal_len;
        diff = ps_l - ps_r;
        //std::cout << diff << std::endl;
        //ps_vals;
        //robot->step(1);
        if (diff <= -0.6)
        {
          stop();
          //std::cout << "Restored Position!!" << std::endl;
          //ps_vals;
          break;
        }
      }
    }
    return ret_val;
  }
  
  void go(double x, double speed,PositionSensor *ps_right,PositionSensor *ps_left,Robot *robot,int timeStep) // go x(m) distance forward
  {
    double ps_r = ps_right -> getValue();
    double ps_l = ps_left -> getValue();
    float radius = 0.03;
    double disError = 0.002*speed;
    double refVal = ps_right->getValue();
    double val, dis;
    moveFwd(speed);
    while (robot->step(timeStep) != -1) 
    {
      val = ps_right->getValue() - refVal;
      dis = val*radius;
      //std::cout << dis <<" m"<< std::endl;
      if (dis>=x-disError)
      {
        stop();
        //std::cout << "Stopped!!"<< std::endl;
        break;
      }
    }
  }
  
  void moveLeft(int speed)
  {
    forward(-speed,speed);
  }
  
  void moveRight(int speed)
  {
    forward(speed,-speed);
  }
  void moveFwd(int speed)
  {
    forward(speed,speed);
  }
  void moveBwd(int speed)
  {
    forward(-speed,-speed);
  }
    
}

