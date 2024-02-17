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

namespace sithuruwan
{
  void lift_ds_panel(Robot *robot,Motor *motors[5])
  {
    motors[4]->setVelocity(5.0);
    motors[4]-> setPosition(0.03);
    robot->step(500);
    motors[4]->setVelocity(0.0);

  }

  void reset_ds_panel(Robot *robot,Motor *motors[5])
  {
    motors[4]->setVelocity(5.0);
    motors[4]-> setPosition(0);
    robot->step(500);
    motors[4]->setVelocity(0.0);
  
  }

  int function2(int down,int up)
  {
    
    if ((250<up && up<550)||(650<up && up<950)||(1050<up && up<1350))
    {
      return 2*4*round(up/400.0);
    }
    else if((250<down && down<550)||(650<down && down<950)||(1050<down && down<1350))
    {
      return 1*4*round(down/400.0);
    }
    else
    {
      return -1;
    }
  }



  int Val_of_N(vector<int> distances)
  {
    int Left_Value=function2(distances[0],distances[2]);
    int Right_Value=function2(distances[1],distances[3]);
   // cout<<Left_Value<<Right_Value<<endl;
    if(Left_Value==-1 or Right_Value==-1)
    {
      return -1;
    }
    else
    {
      cout<<"Right: "<<Right_Value<<" Left: "<<Left_Value<<endl;
      int the_val =  abs(Left_Value-Right_Value);
      //cout<<the_val<<endl;
      if ((the_val == 4) || (the_val == 8) || (the_val == 12))
      {
        return the_val;
      }
      else
      {
        return -1;
      }
    
    
    }
  
  }

  vector<int> Proper_factors(int x)
  {
    vector<int> OUTPUT;
    int root=sqrt(x)+1;
    vector<int> factors;
    int k=0;
    for(int i=1;i<root;i++)
    {
      if((x%i)==0)
      {
        int y=(int) (x/i);
        factors.push_back(i);
        factors.push_back(y);
        k=k+2;
      }
    }
    int lenth=factors.size();
    for(int j=0;j<k;j=j+2)
    {
      OUTPUT.push_back(factors[j]);
    }
    if (lenth%2)
    {
      for(int j=lenth-2;j>0;j=j-2)
      {
        if ((j==lenth-2) && (factors[j]==factors[j+1]))
        {
          continue;
        }
        OUTPUT.push_back(factors[j]);
      }
    }
    else
    {
      for(int j=lenth-1;j>0;j=j-2)
      {
      if ((j==lenth-1) && (factors[j]==factors[j-1]))
      {
        continue;
      }
      OUTPUT.push_back(factors[j]);
      }
    }
    return OUTPUT;
  }

  int optional_task(DistanceSensor *ds[3],Robot *robot,Motor *motors[5])
  {
    robot->step(500);
    vector<int> distances;
    distances.push_back(ds[1]->getValue()-150);
    distances.push_back(ds[2]->getValue()-150);
    lift_ds_panel(robot,motors);
    distances.push_back(ds[1]->getValue()-150);
    distances.push_back(ds[2]->getValue()-150);
    reset_ds_panel(robot,motors);
    return Val_of_N(distances);
    
  }
    
}

