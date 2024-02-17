#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
//#include <webots/Keyboard.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <fstream>

#include "AllHeaders.h"

using namespace std;
using namespace webots;

//////////////////////////////Variables
double base_1[] = {1,0.085,0.2};
double base_2[] = {2,0.085,0.2};

vector<double> base_3 = {3,0.085,0.2};
vector<double> base_4 = {4,0.092,0.22};
vector<vector<double>> config = {base_3,base_4} ;

double kp;
double kd;
double baseSpeed;
int width;
int timeStep;
int junction_count = 0;
int option;
double start_time;
double end_time;
double baseVal_ren = 0;
double baseVal_len = 0;
vector<string> wheelnames = {"leftMotor","rightMotor"};
vector<string> motornames = {"camera_slider","camera_servo_y","camera_servo_x","surface_slider","ds_slider"};
vector<string> dsnames = {"ds_front","ds_left","ds_right"};
int box_color[3];
string color ;
int end_box_code = 0;
bool found_color_box = 0;
int turn_step_count = 0;
bool error_found = 0;
int error_count = 0;
////////////////////////////Objects

Motor *wheels[2];
Motor *motors[5];
DistanceSensor *ds[3];
Robot *robot;
Camera *camera;
PositionSensor *ps_right;
PositionSensor *ps_left;


void forward(double ls,double rs)
{
  if(abs(ls) > 10){
      ls = (ls/abs(ls))*10;
    }
    if(abs(rs) > 10){
      rs = (rs/abs(rs))*10;
    }

  wheels[0]->setVelocity(ls);
  wheels[1]->setVelocity(rs);
}
void stop()
{
  wheels[0]->setVelocity(0);
  wheels[1]->setVelocity(0);
}

string color_box()
{
  const unsigned char *image = camera->getImage();
  string color_of_box = "";
  vector <vector <vector <int>>> color_pixels = {{},{},{}};
  for (int y = 2; y < 31; y++)
  {
    for (int x = 5; x < 59; x++)
    {
      int r = camera->imageGetRed(image, width, x, y);
      int g = camera->imageGetGreen(image, width, x, y);
      int b = camera->imageGetBlue(image, width, x, y);
      if ((r>g) && (r>b))
      {
        vector <int> cords = {x,y};
        color_pixels[0].push_back(cords);
      }
      else if ((g>r) && (g>b))
      {
        vector <int> cords = {x,y};
        color_pixels[1].push_back(cords);
      }
       else if ((b>r) && (b>g))
      {
        vector <int> cords = {x,y};
        color_pixels[2].push_back(cords);
      }
    }

    if (((color_pixels[2].size() == 0 ) && (color_pixels[1].size() == 0 )) || ((color_pixels[0].size() > color_pixels[2].size()) && (color_pixels[0].size() > color_pixels[1].size())))
    {
      color_of_box = "Red";
    }
    else if (((color_pixels[0].size() == 0 ) && (color_pixels[2].size() == 0 )) || ((color_pixels[1].size() > color_pixels[2].size()) && (color_pixels[1].size() > color_pixels[0].size())))
    {
      color_of_box = "Green";
    }
     else if (((color_pixels[0].size() == 0 ) && (color_pixels[1].size() == 0 )) || ((color_pixels[2].size() > color_pixels[0].size()) && (color_pixels[2].size() > color_pixels[1].size())))
    {
      color_of_box = "Blue";
    }

  }
  return color_of_box;
}

vector <vector <vector <vector <int>>>> image_spliting(const unsigned char *image)///Code to 
{
  vector <vector <vector <int>>> white_pixels;
  vector <vector <vector <int>>> color_pixels;
  int count = 0;
  for (int y = 38; y < 64; y++)
  {
    white_pixels.push_back({});
    if (count<16)
    {
      color_pixels.push_back({});
    }
    for (int x = 0; x < 64; x++)
    {
      int r = camera->imageGetRed(image, width, x, y);
      int g = camera->imageGetGreen(image, width, x, y);
      int b = camera->imageGetBlue(image, width, x, y);
      int grey = (r+b+g)/3;
      if (r>200 && g>200 && b>200)
      {
        vector <int> cords = {x,y};
        white_pixels[count].push_back(cords);
      }
      else if ((count<16)&& (y<54) &&(abs(grey - r) >10 or abs(grey - g) >10 or abs(grey - b) >10) )
      {
        vector <int> cords = {x,y};
        color_pixels[count].push_back(cords);
      }
    }
    count += 1;
  }
  vector <vector <vector <vector <int>>>> the_return= {white_pixels,color_pixels};
  return the_return;
}

void go_on_line(int baseSpeed,vector<vector <vector <int>>> line,int previous_error)
{
  int point = 0;
  int count_lines = 0;
  for (unsigned int i = 0;i<line.size();i++)
  {
    if (line[i].size()>0)
    {
      point += (line[i][0][0] + line[i][line[i].size()-1][0])/2;
      count_lines ++;
    }
  }
  point = point/count_lines;
  int error = (point-31);
  int diff_error = error - previous_error;
  double velocity_correction = error*kp + diff_error*kd;
  double ls = baseSpeed + velocity_correction;
  double rs = baseSpeed - velocity_correction;
  forward(ls,rs);
}

int box_finding()
{
  motors[0]->setVelocity(5);
  motors[0]-> setPosition(0.065);
  robot->step(500);
  string returned_color = color_box();
  if (returned_color == color)
  {
    end_box_code = 8;
  }
  else
  {
    motors[1]->setVelocity(5);
    motors[1]->setPosition(-1.2);//-1.21428
    robot->step(500);
    string returned_color = color_box();
    if (returned_color == color)
    {
      end_box_code = 12;
    }
    else
    {
      end_box_code = 4;
    }
  }
  motors[1]->setPosition(0);
  motors[0]-> setPosition(0);
  robot->step(500);
  motors[0]->setVelocity(0);
  motors[1]->setVelocity(0);

  cout<<end_box_code<<endl;
  return end_box_code;

}

void turn_left()
{
  dumindu::turnAng(-90,ps_left,ps_right,wheels,robot);
  baseVal_ren = ps_right -> getValue();
  baseVal_len = ps_left -> getValue();
}
void turn_right()
{
  dumindu::turnAng(90,ps_left,ps_right,wheels,robot);
  baseVal_ren = ps_right -> getValue();
  baseVal_len = ps_left -> getValue();
}
void turning(double baseSpeed_change,int previous_error)
{
  while (robot->step(timeStep) != -1)
  {
    const unsigned char *image = camera->getImage();
    vector <vector <vector <int>>> white_pixels;
    vector <vector <vector <vector <int>>>> the_return = image_spliting(image);
    white_pixels = the_return[0];
    vector<vector <vector <int>>> line = std::vector<vector <vector <int>>>(white_pixels.begin() + 22, white_pixels.end());
    vector<vector <vector <int>>> junction = std::vector<vector <vector <int>>>(white_pixels.begin(),white_pixels.end()-11);
    go_on_line(baseSpeed_change,line,previous_error);
    if (junction[0].size()>35 && junction[junction.size()-1].size()>=20 )
    {
      if ( (ds[1]->getValue() < 1500 ) && (ds[2]->getValue() < 1500))
      {
        if (option == 1)
        {
          limalka::go(0.07,baseSpeed,ps_right,ps_left,robot,timeStep);
          end_box_code = sithuruwan::optional_task(ds,robot,motors);
          if (end_box_code>0)
          {
            cout<<end_box_code<<endl;
            stop();
            limalka::go(0.1,baseSpeed,ps_right,ps_left,robot,timeStep);
          }
          else
          {
            option = 2;
            cout<<"Cant calculate or wrong number received "<<endl;
            cout<<"Skipping optional task"<<endl;
            limalka::go(0.1,baseSpeed,ps_right,ps_left,robot,timeStep);
          }
        }
        else if(option == 2)
        {
          limalka::go(0.17,baseSpeed,ps_right,ps_left,robot,timeStep);
        }
        break;
      }
    }
    else
    {
      stop();
      
      int j_point = 0;
      for (unsigned int i = 0;i<junction.size();i++)
      {
        j_point += (junction[i][0][0] + junction[i][junction[i].size()-1][0]);
      }
      j_point = j_point/32;
      if (j_point-31 >6)
      {
        limalka::go(0.08,baseSpeed,ps_right,ps_left,robot,timeStep);
        turn_right();
        
       
      }
      else if (31 - j_point >6)
      {
        limalka::go(0.08,baseSpeed,ps_right,ps_left,robot,timeStep);
        turn_left();
        
        
        
      }
      else
      {
        if(junction_count == 0)
        {
          limalka::go(0.08,baseSpeed,ps_right,ps_left,robot,timeStep);
          if (option == 2)
          {

            turn_right();
          
           
            
          }
          else
          {
            turn_left();
            
           
            
          }


        }
        else if (junction_count == 1)
        {
          int returned;
          if ((option == 1) && ((end_box_code == 4) || (end_box_code == 8) || (end_box_code == 12) ))
          {
            returned = end_box_code;
          }
          else
          {
          returned = box_finding();
          end_box_code = returned;
          }
          if (returned == 4)
          {
            limalka::go(0.08,baseSpeed,ps_right,ps_left,robot,timeStep);
            turn_left();
           
          }
          else if(returned == 12)
          {
            limalka::go(0.08,baseSpeed,ps_right,ps_left,robot,timeStep);
            turn_right();
           
          }
          else
          {
            limalka::go(0.09,baseSpeed,ps_right,ps_left,robot,timeStep);
          }
        }
        else
        {
          error_found =1;
          error_count ++;
          break;
          
        }
        junction_count += 1;
      }

      break;
    }
  }
}

void linefollowing()
{
  int previous_error = 0;
  while (robot->step(timeStep) != -1)
  {
    const unsigned char *image = camera->getImage();
    vector <vector <vector <int>>> white_pixels;
    vector <vector <vector <int>>> color_pixels;
    vector <vector <vector <vector <int>>>> the_return = image_spliting(image);
    if (the_return[0][25].size()>0)//0
    {
      white_pixels = the_return[0];
      vector<vector <vector <int>>> line = std::vector<vector <vector <int>>>(white_pixels.begin() + 22, white_pixels.end());
      vector<vector <vector <int>>> junction = std::vector<vector <vector <int>>>(white_pixels.begin(),white_pixels.end()-11);
 
      if (line.size()>0)
      {
        go_on_line(baseSpeed,line,previous_error);
        if (junction[0].size()>35 && junction[junction.size()-1].size()>=20)
        {
          turning(base_1[0],previous_error);
        }
      }

    }
    else if (the_return[1][0].size() == 0 && the_return[0][25].size() == 0)
    {
      stop();
      cout<<"Line out of sight attempting correction"<<endl;
      double ret_val = limalka::reset(1,ps_right,ps_left,robot,timeStep,baseVal_ren,baseVal_len);
      error_count ++;
      error_found = 1;
      stop();
      if (ret_val>0)
      {
        dumindu::turnAng(-45,ps_left,ps_right,wheels,robot); 
      }
      else
      {
        dumindu::turnAng(45,ps_left,ps_right,wheels,robot);
      }
      double find_start_time = robot->getTime();
      double find_current_time;
      while (robot->step(timeStep) != -1)
      {
        const unsigned char *image = camera->getImage();
        vector <vector <vector <int>>> white_pixels;
        vector <vector <vector <vector <int>>>> the_return = image_spliting(image);
        if (the_return[0][0].size()>0)
        { 
        white_pixels = the_return[0];
        vector<vector <vector <int>>> line = std::vector<vector <vector <int>>>(white_pixels.begin() + 22, white_pixels.end());
        if (line[3].size()>20)
        {
          stop();
          break;
        }
         else
        {
          forward(1,1);
        } 
        }
        else
        {
          forward(1,1);
        }
        find_current_time = robot->getTime();  
      if ((find_current_time-find_start_time)>20)
      {
        break;
      }
      }
      if (error_count>1)
      {
        break;
      }
      

    }
    

    if ((ds[1]->getValue()<500 && ds[2]->getValue()<500) && (option == 2))
    {
      cout<<"At wall following"<<endl;
      stop();
      break;
    }
    
    if (the_return[1][0].size()>0)
    {
      color_pixels = the_return[1];
      if (color_pixels[0].size()>0 && color_pixels[color_pixels.size()-1].size()>0 )
      {
        if (found_color_box== 0)
        {
          stop();
          double r = 0;
          double g = 0;
          double b = 0;
          double count = 0;
          unsigned int row = color_pixels.size();
  
          for (unsigned int i=0; i<row;i++)
          {
            unsigned int column = color_pixels[i].size();
            for (unsigned int j=0;j<column;j++)
            {
              r += camera->imageGetRed(image, width, color_pixels[i][j][0], color_pixels[i][j][1]);
              g += camera->imageGetGreen(image, width, color_pixels[i][j][0], color_pixels[i][j][1]);
              b += camera->imageGetBlue(image, width, color_pixels[i][j][0], color_pixels[i][j][1]);
              count += 1;
              if ((row/2 == i) && (column/2 == j))
              {
                box_color[0] = camera->imageGetRed(image, width, color_pixels[i][j][0], color_pixels[i][j][1]);
                box_color[1] = camera->imageGetGreen(image, width, color_pixels[i][j][0], color_pixels[i][j][1]);
                box_color[2] = camera->imageGetBlue(image, width, color_pixels[i][j][0], color_pixels[i][j][1]);
              }
            }
          }
          if (r/count > g/count && r/count> b/count)
          {
            color = "Red";
          }
          else if (g/count > r/count && g/count> b/count)
          {
            color = "Green";
          }
          else
          {
            color = "Blue";
          }
          
          cout<<color<<endl;
          limalka::go(0.17,baseSpeed,ps_right,ps_left,robot,timeStep);
          found_color_box = 1;
        }
  
       else
       {
         limalka::go(0.11,baseSpeed,ps_right,ps_left,robot,timeStep);
         cout<<"Done"<<endl;
         break;
       }
      }
    }
   
  };
}

int main(int argc, char **argv)
{

  robot = new Robot();
  timeStep = 32;
  camera= robot->getCamera("camera");
  camera->enable(timeStep);
  width = camera->getWidth();
  ps_right = robot ->getPositionSensor("rightencorder");
  ps_left = robot ->getPositionSensor("leftencorder");
  ps_right -> enable(timeStep);
  ps_left -> enable(timeStep);
  robot->step(100);
  for (int i = 0; i < 5 ; i++)
  {
    motors[i] = robot->getMotor(motornames[i]);
    motors[i]-> setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }

  for (int i = 0; i < 2 ; i++)
  {
    wheels[i] = robot->getMotor(wheelnames[i]);
    wheels[i]-> setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }

  for (int i = 0; i < 3 ; i++)
  {
    ds[i] = robot->getDistanceSensor(dsnames[i]);
    ds[i]->enable(timeStep);

  }
  
  ifstream file ("Config.txt");
  string numm;
  getline(file,numm);
  int k = stoi(numm);
  cout<<"Config value is "<<k<<" ,if this is the first attempt set Config.txt file value to 0"<<endl;
  
  file.close();
  
   int choice_1,choice_2;
  if (k ==0)
  {
    choice_1 = 4;
    choice_2 = 1;
  }
  else if (k == 1)
  {
    choice_1 = 4;
    choice_2 = 2;
  }
  else
  {
    choice_1 = 3;
    choice_2 = 2;
  }
  
  kp = config[choice_1-3][1];
  kd = config[choice_1-3][2] ;
  baseSpeed = config[choice_1-3][0];
  option = choice_2;
  //start_time = robot->getTime();
  //cout<<"Start time " <<start_time<<endl;
  baseVal_ren = ps_right -> getValue();
  baseVal_len = ps_left -> getValue();
  limalka::go(0.05,baseSpeed,ps_right,ps_left,robot,timeStep);

  linefollowing();
  if ((option == 2) && (error_count<2))
  {
    dumindu::wallFollow(ds,robot,baseSpeed);
    linefollowing();
  }
  
  if (error_found==1)
  {
    stop();
    if (k==2)
    {
      k = 0;
    }
    else
    {
      k++;
    }
    ofstream file_1 ("Config.txt");
  
    file_1<< to_string(k);
    file_1.close();
  
  }
  cout<<"End box code: "<<end_box_code<<endl;
  vector<int> factors_of_N=sithuruwan::Proper_factors(end_box_code);

  for(unsigned long long int j=0;j<factors_of_N.size();j++)
  {
    cout<<factors_of_N[j]<<endl;
  }
  end_time = robot->getTime();
  cout<<"End time " <<end_time<<endl;
  
  //cout<<"Timing " <<end_time - start_time<<endl;
  
  
  delete robot;
  return 0;
}
