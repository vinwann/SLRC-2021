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
vector<vector<double>> scan_modes = {{-0.0698132,1.68806},{-0.95806,1.68806},{-1.68806,1.68806},{-1.68806,0.95806},{-1.68806,0.0698132}};
namespace lidar_methods

{
  vector<vector<double>> scan_for_objects(DistanceSensor *lidar,Motor *motors[6],Robot *robot,vector<int> current_node,int dir)
{
  vector<double> scan_area = scan_modes[2];
  if(dir == 1 || dir == 3)
  {
    int swap_val = 0;
    if (dir==3)
    {
      swap_val = 4;
    }
    if (current_node[1] == 0)
    {
      scan_area = scan_modes[abs(swap_val-0)];
    }
    else if(current_node[1] == 6)
    {
      scan_area = scan_modes[abs(swap_val-4)];
    }
    else if(current_node[1] == 1)
    {
      scan_area = scan_modes[abs(swap_val-1)];
    }
    else if(current_node[1] == 5)
    {
      scan_area = scan_modes[abs(swap_val-3)];
    }
  }
  else
  {
    int swap_val = 4;
    if (dir==2)
    {
      swap_val = 0;
    }
    if (current_node[0] == 2)
    {
      scan_area = scan_modes[abs(swap_val-0)];
    }
    else if(current_node[0] == 10)
    {
      scan_area = scan_modes[abs(swap_val-4)];
    }
    else if(current_node[0] == 3)
    {
      scan_area = scan_modes[abs(swap_val-1)];
    }
    else if(current_node[0] == 9)
    {
      scan_area = scan_modes[abs(swap_val-3)];
    }
  }
  double count = scan_area[0];//scan_modes[current_node[0]][0];
  //motors[0]->setVelocity(7);
 // motors[0]-> setPosition(count);
  set_lidar_pos(count,7,motors);
  robot->step(2000);
  vector<vector<double>> values = {};
  vector<vector<double>> out = {};
  double dis = lidar->getValue();
  if (dis<1500)
  {
    values.push_back({dis,count});
    //cout<<dis<<" "<<count<<endl;
  }
  while (count<scan_area[1])//scan_modes[current_node[0]][1])
 {
  count +=0.01745;
  //motors[0]-> setPosition(count);
  //motors[0]->setVelocity(1.090);
  set_lidar_pos(count,1.090,motors);
  robot->step(32);
  double dis = lidar->getValue();
  if (dis<1500)
  {
    values.push_back({dis,count});
  }
  };
  if (values.size()==0)
  {
    return values;
  }
  
  cout<<current_node[0]<<current_node[1]<<dir<<endl;
  for (int i = 0; i < values.size() ; i++)
  { 
    //cout<<values[i][0]*cos(values[i][1])/600<<" "<<values[i][0]*sin(values[i][1])/600<<endl;
    
    double x = round(round(values[i][0]*cos(values[i][1])/60)/10);//+ current_node[1];
    double y = round(round(values[i][0]*sin(values[i][1])/60)/10);// + current_node[0];
    //cout<<robot->getName()<<" "<<x<<" "<<y<<endl;
    if (dir == 1)
    {
      y += current_node[0];
      x += current_node[1];
    }
    else if (dir == 3)
    {
      y -= current_node[1];
      x -= current_node[0];
      x = abs(x);
      y = abs(y);
    }
    else if (dir == 0)
    {
      //cout<<"hereee"<<endl;
      int temp = y;
      y = current_node[1] + x;
      y = abs(y);
      x = current_node[0] - temp;
      x = abs(x);
      //cout<<x<<y<<endl;
    }
    else 
    {
      int temp = y;
      y  = current_node[1] - x;
      x  = current_node[0] + temp;
    }
    vector<double> val = {x,y};
    
    if (out.size()== 0)
    {
      out.push_back(val);
    }
    else
    {
      int num_of = 0;
      for (int i = 0; i < out.size() ; i++)
      {
        if (val == out[i])
        {
          num_of +=1;
        }
      }
      if (num_of == 0)
      {
        out.push_back(val);
      }

    }
  }
  set_lidar_pos(0,7,motors);
  //->step(500);
  
  
  return out;
}
vector<double> look_for_box(DistanceSensor *lidar,Motor *motors[6],Robot *robot)
{
  double count =-1.5708;
  //motors[0]->setVelocity(7);
  //motors[0]-> setPosition(-1.57);
  set_lidar_pos(-1.5708,7,motors);
  robot->step(2000);
  vector<vector<double>> values = {};
  vector<vector<double>> out = {};
  double dis = lidar->getValue()+15;
  
  if (dis<400)
  {
    values.push_back({dis,count});
    //cout<<dis<<" "<<count<<endl;
  }
  while (count< 1.5708)
 {
  
  //motors[0]-> setPosition(count);
  //motors[0]->setVelocity(1.090);
  set_lidar_pos(count,1.090,motors);
  count +=0.01745;
  robot->step(32);
  double dis = lidar->getValue()+15;
  
  if (dis<400)
  {
    values.push_back({dis,count});
    //cout<<dis<<" "<<count<<endl;
  }
  //cout<<ps_lidar->getValue()<<endl;
  //cout<<robot->getTime()<<" "<<count<<endl;
  
  };
  if (values.size()>0)
  {
  double x1 = values[0][0]*sin(values[0][1])/10 ;
  double y1 = values[0][0]*cos(values[0][1])/10;
  double x2 = values[values.size()-1][0]*sin(values[values.size()-1][1])/10;
  double y2 = values[values.size()-1][0]*cos(values[values.size()-1][1])/10;
  double act_y = (y2+y1)/2;
  double act_x = 0;
  if (x2>=0 && x1<0)
  {
   act_x = (x1+x2)/2 ;//+(abs(x1)+x2)/2; 
   
  }
  else
  {
    act_x = (x2+x1)/2;
  } 
  //cout<<values[0][0]<<" "<<values[0][1]<<" "<<x2<<" "<<x1<<" "<<x2-x1<<" "<<(y2+y1)/2<<endl;
  //cout<<act_y<<" "<<act_x<<endl;
  set_lidar_pos(0,7,motors);
  return {act_x,act_y};
  }
  else
  {
    set_lidar_pos(0,7,motors);
    return {};
  }
 }
 void set_lidar_pos(double val,double vel,Motor *motors[6])
{
  motors[0]->setVelocity(vel);
  motors[0]-> setPosition(val);
}
bool check_road(DistanceSensor *lidar,Motor *motors[6],Robot *robot)
{
   double count =-0.1709;
  set_lidar_pos(count,7,motors);
  robot->step(600);
  double dis = lidar->getValue();
  if (dis<700)
  {
    set_lidar_pos(0,7,motors);
    return true;
    //cout<<dis<<" "<<count<<endl;
  }
  while (count<0.30608)
 {
  count +=0.01745;
  //motors[0]-> setPosition(count);
  //motors[0]->setVelocity(1.090);
  set_lidar_pos(count,1.090,motors);
  robot->step(32);
  double dis = lidar->getValue();
  if (dis<700)
  {
    set_lidar_pos(0,7,motors);
    return true;
  }
  };
  set_lidar_pos(0,7,motors);
  return false;
}

}

