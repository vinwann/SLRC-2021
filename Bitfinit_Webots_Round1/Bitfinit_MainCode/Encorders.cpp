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

namespace encoders
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
    double disError = 0;//.002*speed;
    double refVal = ps_right->getValue();
    double val, dis;
    moveFwd(speed);
    while (robot->step(timeStep) != -1) 
    {
      val = ps_right->getValue() - refVal;
      dis = val*radius;
      //std::cout << dis <<" m"<< std::endl;
      if (abs(dis)>=x-disError)
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
  
  void setLeftSpeed(double speed,Motor *wheels[2])
   {
    wheels[0]->setVelocity(speed);
    
   }
  
  void setRightSpeed(double speed,Motor *wheels[2])
  {
    wheels[1]->setVelocity(speed);
   
  }
  double roundTo(double number, int n)
  {
    number = number * pow(10,n);
    number = round(number);
    number = number / pow(10,n);
    return number;
  }  
 


 void turnAng(double angle,PositionSensor *ps_left,PositionSensor *ps_right,Motor *wheels[2],Robot *robot,int timeStep)
 {
  PositionSensor *ps[2] = {ps_left,ps_right};
  int side = 0;
  if(angle < 0){
    side = 1;
    angle = -angle;
  }
  double init_val = ps[side]->getValue();
  while (robot->step(timeStep) != -1) 
    {
      if (side == 0)
      {
        moveRight(3);
      }
      else
      {
        moveLeft(3);
      }
    double val = ps[side]->getValue();
    //cout<<val<<init_val<<endl;
    if((val-init_val)*0.03>=2*0.2*1.873*(angle/360))
    {
      moveFwd(0);
      break;
    }
  }; 
  }    
  /*
  int s = 1;
  int p = 0;
  int gear = 0;
  double part,speed;
  
  if(angle < 0){
    s = -1;
    p = 1;
    angle = -angle;
  }
  angle = roundTo(3.078/90 * angle,3);
 
  double pos1 = ps[p]->getValue();
  
  while(roundTo((ps[p]->getValue() - pos1),3) < angle){
    part = roundTo((ps[p]->getValue() - pos1)/angle*100,3);
    gear = (part / 25) + 1;
    speed = (5-gear);
    
    setLeftSpeed(s*speed,wheels);
    setRightSpeed(-s*speed,wheels);
    robot->step(1);
  }
  setLeftSpeed(0.0,wheels);
  setRightSpeed(0.0,wheels);
  } 
  */  
}

