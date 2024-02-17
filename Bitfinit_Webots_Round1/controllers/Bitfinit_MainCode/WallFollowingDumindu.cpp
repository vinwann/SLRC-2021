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
#include <cmath>

using namespace std;

namespace dumindu
{
   void setLeftSpeed(double speed,Motor *wheels[2])
   {
    wheels[0]->setVelocity(speed);
    
   }
  
  void setRightSpeed(double speed,Motor *wheels[2])
  {
    wheels[1]->setVelocity(speed);
   
  }


// Method to round a double to a specific decimal places -- (params - double value,decimal places)
  double roundTo(double number, int n)
  {
    number = number * pow(10,n);
    number = round(number);
    number = number / pow(10,n);
    return number;
  }  
 


 void turnAng(double angle,PositionSensor *ps_left,PositionSensor *ps_right,Motor *wheels[2],Robot *robot)
 {
  PositionSensor *ps[2] = {ps_left,ps_right};

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
  
  void wallTurn(DistanceSensor *ds[3])
  {
    if(ds[1]->getValue() < ds[2]->getValue())
    {
      turn_right();//turnRight();
    }
    else
    {
    turn_left();//turnLeft();
    }
    forward(3,3);
  }
  void wallFollow(DistanceSensor *ds[3],Robot *robot,double baseSpeed = 4){
    double k1,k2,sp,vControl,error,pre_error,diff_control,error_diff,pre_sp;
    int side = 0,state = 0,pre_state;
    double speedL,speedR;
    bool sideSet = false,stateChanged = false;
    sp = 3.0;
    k1 = 1.2;
    k2 = 2.0;
    pre_error = 0.0;
  while(true){
    if(ds[1]->getValue() > 1500 and ds[2]->getValue() > 1500){
     
      stop();
      break;
    }
    
    if(ds[1]->getValue() < 505 and ds[2]->getValue() < 505){
   
      pre_state = state;
      state = 2;
    }else{
    
      pre_state = state;
      state = 1;
    }
    
    
    if(pre_state != state){
      
      stateChanged = true;
    }else{
      stateChanged = false;
    }
    
    if(stateChanged and state == 2){
      
   
      pre_sp = sp;
      sp = roundTo((ds[1]->getValue() + ds[2]->getValue())/200,1);
      if(sp > 6.0){
        sp = pre_sp;
      }
    }
    
    if(roundTo(ds[0]->getValue(),2) < 400){
      
      stop();
      wallTurn(ds);
      sp = 3.0;
    }
    
    if(ds[1]->getValue() > 1500 or ds[2]->getValue() > 1500){
      sideSet = false;
    }
    
    if(ds[1]->getValue() > 1500 and ds[2]->getValue() < 600){
      
      side = 1;
      sideSet = true;
    }else if(not sideSet){
     
      side = 0;
    }
    error = roundTo(sp - ds[side + 1]->getValue()/100,2);
    error_diff = pre_error - error;
   
    
    vControl = error*k1;
    diff_control = error_diff*k2;
    speedL = (baseSpeed +(1-2*side)*vControl -(1-2*side)*diff_control);
    speedR = (baseSpeed -(1-2*side)*vControl +(1-2*side)*diff_control);
    
    
    
    if(abs(speedL) > 10){
      speedL = (speedL/abs(speedL))*10;
    }
    if(abs(speedR) > 10){
      speedR = (speedR/abs(speedR))*10;
    }
    
   
    forward(speedL, speedR);
    robot->step(10);
  }
}  
  
  
}