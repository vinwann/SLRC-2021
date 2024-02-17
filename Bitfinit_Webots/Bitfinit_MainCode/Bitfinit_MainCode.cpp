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
#include <cstring>
#include <algorithm>
#include "AllHeaders.h"
#include <webots/Lidar.hpp>
using namespace std;
using namespace webots;
#include <math.h>
#include <sstream>
//////////////////////////////Variables
//////////////////////////////////////////////////////new default
string my_loc_file = "";
string other_loc_file = "";
string my_target_file = "";
string other_target_file = "";
string my_nodes_found = "robot_a.txt";
string other_nodes_found = "robot_b.txt";
vector<vector<int>> targets = {};
int last_read = 0;
vector<int> my_start = {};
vector<int> my_dock = {};
vector<int> other_start = {};
vector<int> other_dock = {};
////////////////////////////////////////////////////////////////////////////////////


double baseSpeed = 4.8;
int width;
int timeStep;
int white_thres = 400;
double baseVal_ren = 0;
double baseVal_len = 0;
bool box_in_front = 0;
double in_box_travel = 0;
int stop_now = 0;
double box_count_at_start = 0;
bool known = false;
bool the_fix = false;
string current_status = "searching";
string currently_carrying = "nothing";
vector<string> wheelnames = {"leftMotor","rightMotor"};
vector<string> gripperdisnames = {"gripper_left","gripper_right"};
vector<string> motornames = {"lidar_servo","linear_motor","j1","j2","g_l","g_r"};
vector<string> irnames = {"ir0","ir1","ir2","ir3","ir4","ir5","ir6","ir7"};
vector<string> camnames = {"camera_left","camera_mid","camera_right","gripper_color_sensor"};
vector<string> priority = {"r","g","b"};
vector<string> current_priority = {"r","g","b"};
vector<string> if_picked = {}; //{"red","green"};
string dock_stat = "w";
string looking_for = "w";//rgb/white
int found_count = 0;
vector<int> to_goal = {};
/////////////////////////////////////////// testing routes
vector<int> pick_up_spot_start = {1};
vector<int> pick_up_spot_dock = {2};
vector<int> start_to_goal = {1};
vector<int> dock_to_goal = {0};
vector<int> dock_to_start = {2};
vector<string> sus_points = {};
vector<string> node = {"1","0000","e","0"};
vector<vector<string>> row = {node,node,node,node,node,node,node};
vector<vector<vector<string>>> map = { row,row,row,row,row,row,row,row,row,row,row,row,row};

/*
map[0][0][0] = "s1";
map[1][0][0] = "1";
map[1][0][1] = "1101";
map[12][6][0] = "s2";
*/
vector<int> target_node = {};
bool inverted_opened = false;
//////////////////////////////////////////////////////////
int direction = 1; 
vector<int> junction = {0,0,0};
vector<int> current_node = {0,0};


//vector<vector<double>> set_scan_modes = {{-0.0698132,1.68806},{-0.523599,1.68806},{0,1.68806},{0,1.68806},{-1.68806,0.523599},{-1.68806,0.0698132}};
int previous_error = 0;
int stop_r = 0;
int miss_count = 0;
////////////////////////////Objects

Motor *wheels[2];
Motor *motors[6];
DistanceSensor *ir[8];
DistanceSensor *gripper_ds[2];
DistanceSensor *lidar;
Robot *robot;
Camera *camera[4];
PositionSensor *ps_right;
PositionSensor *ps_left;
vector<int> nums = {};
////////////////////// Mapping D data ////////////////////////
vector<int> currentPoint = {0,0};
string mapArr[13][7];

int prevPoint[2] = {0,0};
int node_array_index = 0;
int targetNode[2] = {9,3};
int startNode[2] = {0,0};
int h_val_weight = 5;///////////////////////////////////////////////////////////////////////////////////
int g_val_weight = 1;
int currentPathNodes[67][2];
int currentPathPoint = 0;
bool targetReached = false;

int shortPath[67][2];
int shortPathIndex = 0;
int currentShortestPathSteps = 0;

class pathNode{
  public:
    pathNode(){

    }
    pathNode(int a, int b){
      id[0] = a;
      id[1] = b;
      calculate_f_val();
    }
    int id[2];
    int last_node_index = 0;
    int connectedNodes[4][2];
    int nodeType;
    bool nodeOpen = false;
    string blockContent = "e";
    int connectedWeights[4];
    int h_val = 999;
    int g_val = 999;
    int f_val;
    int prevNode[2];
    
    void set_nodeType(int t){
      nodeType = t;
    }
    
    void calculate_f_val(){
      cout<<"creating vals"<<id[0]<<","<<id[1]<<":- "<<endl;
      h_val = abs(id[0] - targetNode[0]) + abs(id[1] - targetNode[1]);
      g_val = abs(id[0] - startNode[0]) + abs(id[1] - startNode[1]);
      f_val = h_val*h_val_weight + g_val*g_val_weight;
      //cout<<h_val<<"|"<<g_val<<"|"<<f_val<<endl;
    }
};

//pathNode n1();
pathNode createdNodes[67];
int openQ[67][2];
int closedQ[67][2];

void turn_back(){
  encoders::turnAng(-180,ps_left,ps_right,wheels,robot,timeStep);
  //Changing directions
  direction+=2;
  direction = direction%4;
  //cout<<"direction"<< " ";
  //cout<<direction<<endl;
}

pathNode * getNode(int a, int b){
  pathNode * node1 = NULL;
  for(int i = 0; i <= node_array_index; i++){
    if(createdNodes[i].id[0] == a and createdNodes[i].id[1] == b){
      node1 = &createdNodes[i];
    }
  }
  return node1;
}

void resetNodes(){
  pathNode * n1;
  for(int i = 0; i <= node_array_index; i++){
    n1 = getNode(createdNodes[i].id[0],createdNodes[i].id[1]);
    n1->calculate_f_val();
  }
}

bool includedInPath(int id0,int id1){
  cout<<"icluded path check"<<id0<<","<<id1<<endl;
  for(int i = 0; i < shortPathIndex; i++){
    if(shortPath[i][0] == id0 and shortPath[i][1] == id1){
      cout<<"true ck"<<endl;
      return true;
    }else{
    cout<<"false ck"<<endl;
     return false;
    }
  }
}

//vector<vector<int>> path = {};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int path[67][3];
//int way_to_got[1] = {};
int fill_array = 0;

void fill_up(vector<int> data)
{
    cout<<"sdf1111111111111111111111111111111111111111111111"<<fill_array<<endl;
    path[fill_array][0] = data[0];
    path[fill_array][1] = data[1];
    path[fill_array][2] = data[2];
    fill_array += 1;
    
}
//string paths= "";
bool check_if_in(int x, int y)
{
  for(int i = 0 ; i<67; i++)
  {
    if (path[i][0] == x && path[i][1] == y)
    {
      return true;
    }
  }
  
  return false;
}

void findShortestPath(int s0, int s1, int t0, int t1,int count)
{ 

  
  pathNode * n1 = getNode(t0,t1);
  
  count += 1;
 
  
  int nextS0, nextS1;
  
  if (n1->nodeOpen == false)
  {
  
    return;
  }
  
  if (check_if_in(t0, t1))
  {
    
    return;
  }
  if(s0==t0 && s1 == t1)
  {
    //cout<<"dfafsa"<<t0<<t1<<count<<endl;//
    
    fill_up({t0,t1,count});
    cout<<"fwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwd"<<s0<<s1<<endl;
    
    return;
  }
  else if ((t0 == 0 && t1 == 0 ) || (t0 == 1 && t1 == 1) || (t0 == 12 && t1 == 6) || (t0 == 11 && t1 == 5))
  {
    return;
    //cout<<"done"<<endl;
  }
  else{
    
   for(int i = 0; i < n1->last_node_index; i++){
   
    nextS0 = n1->connectedNodes[i][0];
    nextS1 = n1->connectedNodes[i][1];
    
    findShortestPath(s0, s1,nextS0,nextS1,count);
    if (check_if_in(t0, t1) == false)
    {
    fill_up({t0,t1,count});
    }
    
   }
  }
  
  
  
}
/*
void go_to_target()
{
  //way_to got
  currentPoint[0];
  path
}
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool nodeExist(int a, int b){
  for(int i = 0; i <= node_array_index; i++){
    if(createdNodes[i].id[0] == a and createdNodes[i].id[1] == b){
      return true;
    }
  }
  return false;
}

bool connectedNodeExist(int a, int b, pathNode * n1){
  for(int i = 0; i < n1->last_node_index; i++){
    if(n1->connectedNodes[i][0] == a and n1->connectedNodes[i][1] == b){
      return true;
    }
  }
  return false;
}

//int

int getNextBestN(){
  int dir1 = 0;
  pathNode * n1 = getNode(currentPoint[0],currentPoint[1]);
  pathNode * n2 = NULL;
  
  //checking if any connected node been visited before and adding higher weight 
  for(int i = 0; i < n1->last_node_index; i++){
    for(int j = 0; j <= currentPathPoint; j++){
      if(n1->connectedNodes[i][0] == currentPathNodes[j][0] and n1->connectedNodes[i][1] == currentPathNodes[j][1]){
        cout<<"visited before, adding weight"<<endl;
        n2 = getNode(currentPathNodes[j][0],currentPathNodes[j][1]);
        n2->f_val += 50;
      }
    }
  }
  
  int minF = 9999999;
  int minFIndex[2];
  for(int i = 0; i < n1->last_node_index; i++){
    cout<<"connected to current node : "<<" ";
    cout<<n1->connectedNodes[i][0]<<",";
    cout<<n1->connectedNodes[i][1]<<" :-";
    //cout<<n1->f_val<<endl;
    
    n2 = getNode(n1->connectedNodes[i][0],n1->connectedNodes[i][1]);
    if(n2->f_val < minF){
      minF = n2->f_val;
      minFIndex[0] = n2->id[0];
      minFIndex[1] = n2->id[1];
    }
    //if(n2->nodeType == 2 or n2->nodeType == 1){
      //n2->f_val += 100;
    //}
    cout<<n2->f_val<<endl;
  }
  cout<<"minF "<<minF<<" minFINdex "<<minFIndex[0]<<","<<minFIndex[1]<<endl;
  
  //same row
  if(currentPoint[1] == minFIndex[1]){
    if(currentPoint[0] < minFIndex[0]){
      dir1 = 1;
    }else{
      dir1 = 3;
    }
  }
  
  //same column
  if(currentPoint[0] == minFIndex[0]){
    if(currentPoint[1] < minFIndex[1]){
      dir1 = 0;
    }else{
      dir1 = 2;
    }
  }
  return dir1;
}

void turnToNextN(){
  int turnDirection = 0;
  turnDirection = getNextBestN();
  cout<<"turn direction"<<turnDirection<<endl;
  
  
  int diff = turnDirection - direction;
  
  if(not( (currentPoint[0] == 0 and currentPoint[1] == 0) or (currentPoint[0] == 1 and currentPoint[1] == 0))){
  if(abs(diff) == 2){
    cout<<"turn back"<<endl;
    turn_back();
  }else if(diff == 1 or diff == -3){
    cout<<"turn right"<<endl;
    turn_right();
  }else if(diff == -1 or diff == 3){
    cout<<"turn left"<<endl;
    turn_left();
  }else{
    cout<<"dont turn"<<endl;
  }
   box_in_front = lidar_methods::check_road(lidar,motors,robot); 
  }
  
  
}

void updateNodes(){
  string dataStr1 = "";
  string dataStr2 = "";
  
  pathNode nt;
  //for x direction ->
  for(int i = 0; i <= 6 ; i++){
    for(int j = 0; j<12; j++){
      dataStr1 = mapArr[j][i];
      dataStr2 = mapArr[j+1][i];
      
      //cout<<"tp"<< typeid(dataStr1[2]).name()<<endl;
      if(dataStr1[2] == '1' or dataStr2[0] == '1'){   
        //Creating and connecting first compared Node
        if(not nodeExist(j,i)){
          nt = pathNode(j,i);
          createdNodes[node_array_index] = nt;
          node_array_index++;
        }
        pathNode * nt1 = getNode(j,i);
        if(not connectedNodeExist(j + 1, i, nt1)){
          nt1->connectedNodes[nt1->last_node_index][0] = j + 1;
          nt1->connectedNodes[nt1->last_node_index][1] = i;
          cout<<nt1->id[0]<<","<<nt1->id[1]<<"<->"<<nt1->connectedNodes[nt1->last_node_index][0]<<","<<nt1->connectedNodes[nt1->last_node_index][1]<<endl;
          nt1->last_node_index++;
        }
        
        
         //Creating and connecting second compared Node
        if(not nodeExist(j + 1,i)){
          nt = pathNode(j + 1,i);
          createdNodes[node_array_index] = nt;
          node_array_index++;
        }
        pathNode * nt2 = getNode(j + 1,i);
        if(not connectedNodeExist(j, i, nt2)){
          nt2->connectedNodes[nt2->last_node_index][0] = j;
          nt2->connectedNodes[nt2->last_node_index][1] = i;
          //cout<<nt.id[0]<<","<<nt.id[1]<<"<->"<<nt.connectedNodes[nt.last_node_index][0]<<","<<nt.connectedNodes[nt.last_node_index][1]<<endl;
          nt2->last_node_index++;
        }
      }
       
    }
  }
  
  
  //for y direction ->
  for(int j = 0; j <= 12 ; j++){
    for(int i = 0; i < 6; i++){
      dataStr1 = mapArr[j][i];
      dataStr2 = mapArr[j][i + 1];
      
      //cout<<"tp"<< typeid(dataStr1[2]).name()<<endl;
      if(dataStr1[1] == '1' or dataStr2[3] == '1'){
        //cout<<"connected"<<" ";
        //cout<<""<<endl;
        
        //Creating and connecting first compared Node
        if(not nodeExist(j,i)){
          nt = pathNode(j,i);
          createdNodes[node_array_index] = nt;
          node_array_index++;
        }
        pathNode * nt1 = getNode(j,i);
        if(not connectedNodeExist(j, i + 1, nt1)){
          nt1->connectedNodes[nt1->last_node_index][0] = j;
          nt1->connectedNodes[nt1->last_node_index][1] = i + 1;
          //cout<<nt.id[0]<<","<<nt.id[1]<<"<->"<<nt.connectedNodes[nt.last_node_index][0]<<","<<nt.connectedNodes[nt.last_node_index][1]<<endl;
          nt1->last_node_index++;
        }
        
         //Creating and connecting second compared Node
        if(not nodeExist(j,i + 1)){
          nt = pathNode(j,i + 1);
          createdNodes[node_array_index] = nt;
          node_array_index++;
        }
        pathNode * nt2 = getNode(j,i + 1);
        if(not connectedNodeExist(j, i, nt2)){
          nt2->connectedNodes[nt2->last_node_index][0] = j;
          nt2->connectedNodes[nt2->last_node_index][1] = i;
          //cout<<nt.id[0]<<","<<nt.id[1]<<"<->"<<nt.connectedNodes[nt.last_node_index][0]<<","<<nt.connectedNodes[nt.last_node_index][1]<<endl;
          nt2->last_node_index++;
        }
      }
       
    }
  }
  
  pathNode * nt3 = getNode(currentPoint[0],currentPoint[1]);
  dataStr1 = mapArr[currentPoint[0]][currentPoint[1]];
  if(dataStr1[4] == '0'){
    nt3->nodeOpen = true;
  }
  nt3->nodeType = dataStr1[4] - '0';
  
  if((nt3->nodeType == 1 && nt3->nodeOpen== false) or nt3->nodeType == 2){
    //cout<<"typecheck11"<<endl;
    nt3->f_val += 500;
  }
  for(int i = 0; i <= node_array_index; i++){
    
      pathNode *node1 = &createdNodes[i];
      cout<<"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"<<node1->id[0]<<" "<<node1->id[1] <<node1->nodeOpen<<endl;
    }
  if(nt3->nodeType == 1){
    cout<<"hererererererer"<<nt3->id[0]<<" "<<nt3->id[1]<< nt3->nodeOpen<<" "<<currently_carrying<<endl;
    if ( nt3->nodeOpen== false)
    {
      if(currently_carrying == "w" ){
      encoders::go(0.23,-1,ps_right,ps_left,robot,timeStep);
        if(junction[0] == 1)
        {
          turn_left();
        }
        else
        {
          turn_right();
        }
        encoders::go(0.01,-2,ps_right,ps_left,robot,timeStep);
        arm_methods::arm_pos(-0.01,1,motors);
        robot->step(1000);
        arm_methods::open_gripper(motors);
        robot->step(1500);
        arm_methods::arm_pos(0.12,4,motors);
        robot->step(1500);
        inverted_opened = true;
        currently_carrying = "nothing";
        encoders::go(0.01,2,ps_right,ps_left,robot,timeStep);
        if(junction[0] == 1)
        {
          turn_right();
        }
        else
        {
          turn_left();
        }
        encoders::go(0.21,1,ps_right,ps_left,robot,timeStep);
        nt3->nodeOpen = true;
        //nt3->f_val -= 500;
        }else{
          cout<<"set prev p"<<" ";
          cout<<prevPoint[0]<<","<<prevPoint[1]<<endl;
          nt3->last_node_index = 1;
          nt3->connectedNodes[0][0] = prevPoint[0];
          nt3->connectedNodes[0][1] = prevPoint[1];
        }
    }
    
    
  }
  
  
  
  for(int i = 0; i < node_array_index; i++ ){
    cout<<createdNodes[i].id[0]<<",";
    cout<<createdNodes[i].id[1]<<" ";
    //cout<<"hv"<<createdNodes[i].h_val<<endl;
  }
  cout<<" "<<endl;
}

void updateMap(int jType){
  //updating robots data
  cout<<direction<<endl;
  
  prevPoint[0] = currentPoint[0];
  prevPoint[1] = currentPoint[1];
  
  if(direction > 1 ){
    if(direction == 3){
      currentPoint[0]--;
    }else{
      currentPoint[1]--;
    }
  }else{
    if(direction == 0){
      currentPoint[1]++;
    }else{
      currentPoint[0]++;
    }
  }
  
  cout<<currentPoint[0]<<" ";
  cout<<currentPoint[1]<<endl;
  
  int current_point_data[5] = {0,0,0,0,0};
  int point_data_index = 0;
  point_data_index += direction;
  for(int i = 0; i <= 2 ; i++){
    current_point_data[point_data_index] = junction[i];
    point_data_index++;
    point_data_index = point_data_index % 4; 
  }
  current_point_data[4] = jType;
  string cpDataString = "";
  for(int a = 0; a <= 4; a ++){
    cpDataString = cpDataString + to_string(current_point_data[a]);
  }
  
  mapArr[currentPoint[0]][currentPoint[1]] = cpDataString;
  //string current_point_data = to_string(junction[0]) + to_string() + to_string() + to_string()
  
   //writing to file //////////////////////////////////////
   
  string id0_str,id1_str;
  id0_str = to_string(currentPoint[0]);
  id1_str = to_string(currentPoint[1]);
  
  vector<string> data1 = {id0_str,id1_str,to_string(current_point_data[0]),to_string(current_point_data[1]),to_string(current_point_data[2]),to_string(current_point_data[3]),to_string(current_point_data[4])};
  
  add_path_nodes(data1);
  
  vector<vector<int>> data2 = get_added_nodes();
  
  cout<<"data2"<<data2.size()<<endl;
  int receivedId[2];
  vector<int> receivedNodeData;
  
  if(data2.size() > 0){
    for(int i = 0; i < data2.size(); i++){
      cout<<"inhhh"<<endl;
      if(mapArr[data2[i][0]][data2[i][1]] == "  x  "){
        cout<<"new point received! "<<endl;
        mapArr[data2[i][0]][data2[i][1]] = to_string(data2[i][2]) + to_string(data2[i][3]) + to_string(data2[i][4]) + to_string(data2[i][5]) + to_string(data2[i][6]);
      }
    }
  }
  
  /////////////////////////////////////////////////////////
      ///////////////////////////// printing mapArr
      for(int i = 6 ; i >= 0; i--){
        for(int j = 0; j <= 12; j++){
         cout<< mapArr[j][i] << " ";
         }
         cout<<""<<endl;  
      }
      ////////////////////////////
      
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------------------path finding algorithm---------------------------------------------------
vector<vector<vector<int>>> solutions;

bool in_arr(int x, int y, vector<vector<int>> arr)
{
  for(int j = 0;j<arr.size();j++)
  {
    if (x == arr[j][0] and y == arr[j][1])
      return true;
  }
  return false;
}

void gen_paths(int a,int b, int x, int y, vector<vector<int>> sol)
{
  cout<<"a "<<a<<"b "<<b<<endl;
  cout<<"len test"<<solutions.size()<<"ids"<<a<<" "<<b<<endl;
  int p,q;
  vector<vector<int>> result = sol;
  vector<int> temp;
  pathNode * start = getNode(a,b);
  cout<<start->id[0]<<",";
  cout<<start->id[1]<<" ";
  cout<<"lni"<<start->last_node_index<<endl;
  //start = getNode(1,0);
  //cout<<"lni2"<<start->last_node_index<<endl;

  for (int i = 0;i<4;i++)
  {
    temp = {0,0};
    p = start->connectedNodes[i][0];
    q = start->connectedNodes[i][1];
    cout<<"ids2 "<<p<<" "<<q<<" "<<endl;
    temp.push_back(p);
    temp.push_back(q);

    if (p == -1 && q == -1){//dead end
      continue;
      }
    if (in_arr(p,q,sol))
    {
      continue;
    }
    result.push_back(temp);
    if (p == x and q == y) // target achived
    {
      //appending the target node to the solution
      temp[0] = x;
      temp[1] = y;
      result.push_back(temp);
      solutions.push_back(result); // append the solution to global array
      return;
    }
    gen_paths(p,q,x,y,result);
  }
}

vector<vector<int>> find_path(vector<vector<vector<int>>> arr)  //search for shortest path
{
  int len_arr = arr.size();
  int min_len = arr[1].size();
  int min_i = 0;
  int len = 0;
  for (int i = 0;i<len_arr;i++)
  {
    len  = arr[i].size();
    if (min_len > len){
      min_len = len;
      min_i = i;
      }
  }
  return arr[min_i];
}

//-----------------------------------------------------------------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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

vector<string> check_for_junction()////Juction check catches red lines as well
{
  vector<string> output ={"0","0","0"};
  for (int i = 0; i < 3 ; i++)
  {
    const unsigned char *image = camera[i]->getImage();
    width = camera[i]->getWidth();
    int r = camera[i]->imageGetRed(image, width, 0, 0);
    int g = camera[i]->imageGetGreen(image, width, 0, 0);
    int b = camera[i]->imageGetBlue(image, width, 0, 0);
    
    if ((r+g+b)/3>180 || r>180)
    {
      output[i] = "1";
    }
    
  }
  
  return output;
}
int check_for_line(int type)
{
  int count = 0;
  for (int i = 0; i < 8 ; i++)
  {
    if (type == 0)
    {
      if (ir[i]->getValue()<400)
      {
        count += 1;
      }
    }
    else
    {
      if (ir[i]->getValue()>400)
      {
        count += 1;
      }
    }
    
  }
  return count;
}
 
int inverted_or_not()////checking if in inverted or not

{
  
  
  vector<string> check = check_for_junction(); 
  
  if (check[0]=="0" && check[2]=="0" ) 
  {
    for (int i = 0; i < 8 ; i++)
    {
      if (ir[i]->getValue()<400)
      {
        return 0;
      }
    
    }
    
  }
  else if  (check[0]=="1" && check[2]=="1" )  
  {
    for (int i = 0; i < 8 ; i++)
    {
      if (ir[i]->getValue()>400)
      {
        return 1;
      }
    
    }
  } 
   
  return 2;
  
}

string get_color()
{
  const unsigned char *image = camera[3]->getImage();
  width = camera[3]->getWidth();
  int r = camera[3]->imageGetRed(image, width, 0, 0);
  int g = camera[3]->imageGetGreen(image, width, 0, 0);
  int b = camera[3]->imageGetBlue(image, width, 0, 0);
  //cout<<"fsadadadadadasfaaaaaaaaaaaf"<<r<<" "<<g<<" "<<b<<" "<<endl;
  if (r>100 && b>100 && g >100)
  {
    return "w";
  }
  else if (r>100 && b<100 && g <100)
  {
    return "r";
  }
  else if (r<100 && b>100 && g <100)
  {
    return "b";
  }
  else if (r<100 && b<100 && g >100)
  {
    return "g";
  }
  return "";
}
vector<string> get_line_color() ////Juction check doesn't catch red lines

{
  vector<string> output ={"0","0","0"};
  for (int i = 0; i < 3 ; i++)
  {
    const unsigned char *image = camera[i]->getImage();
    width = camera[i]->getWidth();
    int r = camera[i]->imageGetRed(image, width, 0, 0);
    int g = camera[i]->imageGetGreen(image, width, 0, 0);
    int b = camera[i]->imageGetBlue(image, width, 0, 0);
    //cout<<"fsadadadadadasfaaaaaaaaaaaf"<<r<<" "<<g<<" "<<b<<" "<<endl;
    if ((r+g+b)/3>180)
    {
      output[i] = "1";
    }
    
  }
  
  return output;
}

string black_line_follow(int previous_error)///Black line following pid
{
  
  double kp = 5;//5;//1;//1.5;
  double kd = 8;//8;//10;//1;
  double baseSpeed = 5;
  
  while (robot->step(timeStep) != -1) 
  {
    
    double line_loc = 0;
    int count = 0;
    
       
    for (int i = 0; i < 8 ; i++)
    {
      if (ir[i]->getValue()>400)
      {
        line_loc += i;
        count += 1;
      }
    }
    if (count >0)
    {
      vector<string> colors = get_line_color();
      vector<string> check_junc = check_for_junction();
      if (check_junc[0] == "0" || check_junc[2] == "0")//(ir[1]->getValue()>400 && ir[2]->getValue()>400 && ir[3]->getValue()>400) ||( ir[4]->getValue()>400 && ir[5]->getValue()>400&& ir[6]->getValue()>400))
      {
         
         robot->step(50);
         stop();
         check_junc = check_for_junction();
         encoders::go(0.04,baseSpeed,ps_right,ps_left,robot,timeStep);
         if (inverted_or_not() == 0)
          {
        
            return "i";
          }
         for (int i = 0; i < 3 ; i++)
        {
        if (colors[i] == "1")
        {
          colors[i] = "0";
        }
        else
        {
          colors[i] = "1";
        }
        }
         
         return colors[0]+colors[1]+colors[2];
      }
      double point= line_loc/count;
      double error = point-3.5;
      double diff_error = error - previous_error;
      double velocity_correction = error*kp + diff_error*kd;
      double ls = baseSpeed + velocity_correction;
      double rs = baseSpeed - velocity_correction;
      if (ls==0)
      {
        ls = 1;
      }
      if (rs==0)
      {
        rs = 1;
      }
      
      previous_error = error;
      forward(ls,rs);
      
      
      }
      else
      {
        
        stop();
        break;
      }
  };
  
  return "";
}

string white_line_follow(int previous_error)///White line following pid goes line segment by segment
{
  
  double kp = 5;//10;//10;
  double kd = 8;//10;
  double baseSpeed = 5;
  double initial_value = (ps_right->getValue() +  ps_left->getValue())/2;
  double current_value = 0 ;
  double distance_traveled = 0;
  while (robot->step(timeStep) != -1) 
  {
    double line_loc = 0;
    int count = 0;
    
       
    for (int i = 0; i < 8 ; i++)///line position calculation algorithm 
    {
      //cout<<i<<" "<<ir[i]->getValue()<<endl;
      if (ir[i]->getValue()<400)
      {
        //cout<<i<<" "<<ir[i]->getValue()<<endl;
        line_loc += i;
        count += 1;
      }
    }
    if (count >0)///If line exists 
    {
    
      vector<string> colors = get_line_color();
      vector<string> check_junc = check_for_junction();
      if (check_junc[0] == "1" || check_junc[2] == "1")///Checking if at juction 
      {
        stop();
        forward(1,1);
        robot->step(100);
        vector<string> check_junc = check_for_junction();
        //cout<<check_junc[0]<<check_junc[1]<<check_junc[2]<<endl;
         stop();
         //cout<<current_node[0]<<" "<<target_node<<endl;
         if ((target_node.size()>0)&&(current_node[0] == target_node[0] && current_node[1] == target_node[1]) && (current_status == "returning") && check_junc[0] == "1" && check_junc[2] == "1")///Change to target node
           {
             return "s";
           }
         if(box_in_front == 0)
         {
           
           //else
           //{
           encoders::go(0.04,baseSpeed,ps_right,ps_left,robot,timeStep);
           //}
         }
         else
         {
           
           return "b";
         }
          if (inverted_or_not() == 1)///Checking if in inverted
          {
        
            return "i";
          }
         return colors[0]+colors[1]+colors[2];
      }
      ///////////////////////pid
      double point= line_loc/count;
      double error = point-3.5;
      
      double diff_error = error - previous_error;
      double velocity_correction = error*kp + diff_error*kd;
      double ls = baseSpeed + velocity_correction;
      double rs = baseSpeed - velocity_correction;
      if (ls==0)
      {
        ls = 1;
      }
      if (rs==0)
      {
        rs = 1;
      }
      //cout<<error<<" "<<ls<<" "<<rs<<endl;
      current_value = (ps_right->getValue() +  ps_left->getValue())/2;
      distance_traveled= (current_value - initial_value)*0.03;
      //cout<<distance_traveled<<endl;
      previous_error = error;
      forward(ls,rs);
      
      
      
    }
      
    else
    {
        stop();
        break;
    }
  };
  return "";
}

void location_update()
{
    if (direction == 0)
      {
        current_node[1] += 1;
      }
      else if (direction == 1)
      {
         current_node[0] += 1;
      }
      else if (direction == 2)
      {
         current_node[1] -= 1;
      }
      else
      {
        current_node[0] -= 1;
      }
}

void junction_code(string junc,int type = 0){///Juction turning code
      //cout<<junc<<endl;
      
      for (int i = 0; i < 3 ; i++)////Updateing junction vector 
      {
        if (junc[i] == '1')
        {
          junction[i] = 1;
        }
        else
        {
          junction[i] = 0;
        }
      }
      cout<<robot->getName()<<" test junc"<<junc<<endl;
      if (known == false)
      {
        cout<<"known1 "<<known<<endl;

      for (int i = 0; i < 3 ; i++)////Updateing junction vector 
      {
        if (junc[i] == '1')
        {
          junction[i] = 1;
        }
        else
        {
          junction[i] = 0;
        }
      }
      
      
      
      int sq = 0;/// Varible to signle that in a sqaure 
      if (junction[1] == 1 && type == 0)
      {
        
        vector<string> colors = get_line_color();
        if (colors[0] == "1" && colors[2] == "1")///Checking for square 
        {
          junction[0] = 0;
          junction[1] = 0;
          junction[2] = 0;
          sq = 1;
        }
        else
        {
          encoders::go(0.035,baseSpeed,ps_right,ps_left,robot,timeStep);
        }
      }
      else
      {
        encoders::go(0.04,baseSpeed,ps_right,ps_left,robot,timeStep);
      }
      //////////////Positioning robot to turn done
      
      int jType = type;
      if(sq == 1){
        jType = 2;
      }
      
      
      
      updateMap(jType);
      updateNodes();
      
      ////shortest path test
      //cout<<"known part"<<endl;
      //gen_paths(0,0,3,0,{});
        
      //cout<<"sol len"<<solutions.size()<<endl;
      //////////////////////
      
      //cout<<"updating path points"<<endl;
      bool visitedBefore;
      
      //cout<<"cpp1 :-"<<currentPathPoint<<endl;
      
      for(int i = 0; i <= currentPathPoint; i++){
        if(currentPathNodes[i][0] == currentPoint[0] and currentPathNodes[i][1] == currentPoint[1]){
          visitedBefore = true;
          //cout<<"visited before"<<endl;
        }else{
          //cout<<"new point"<<endl;
          visitedBefore = false;
        }
      }
      
      if(visitedBefore){
        currentPathPoint--;
      }else{
        //cout<<"creating new points"<<endl;
        currentPathPoint++;
        currentPathNodes[currentPathPoint][0] = currentPoint[0];
        currentPathNodes[currentPathPoint][1] = currentPoint[1];
        
      }
      
      //cout<<"cpp :-"<<currentPathPoint<<endl;
      
      for(int i = 0; i <= currentPathPoint; i++){
        cout<<currentPathNodes[i][0]<<",";
        cout<<currentPathNodes[i][1]<<" ";
      }
      cout<<" "<<endl;
      
      if(currentPoint[0] == targetNode[0] and currentPoint[1] == targetNode[1]){
        cout<<"target reached!"<<endl;
        //pathNode * n1 = getNode(0,0);
        //cout<<"0 con"<<n1->last_node_index<<endl;
        //n1 = getNode(1,0);
        //cout<<"1 con"<<n1->last_node_index<<endl;
        
        //turn_left();
        //turn_left();
        ////////////////////////////////////////////////
        findShortestPath(9,3,1,0,0);
        /////////////////////////////////////////////////////
        known = true;
        current_node = currentPoint;
        //cout<<"dsasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasasa"<<current_node[0]<<current_node[1]<<endl;
        stop();
        //updateMap(0);
        //updateNodes();
        //cout<<"Path size"<<path.size()<<endl;
        
        
        //cout<<"shortest steps"<<paths<<endl;
        
        for(int i = 0; i < shortPathIndex; i++){
          cout<<shortPath[i][0]<<",";
          cout<<shortPath[i][1]<<endl;
        }
                
       /*
        cout<<"cpps"<<currentPathPoint<<endl;
        for(int i = 0; i <= currentPathPoint; i++){
          cout<<"n1"<<currentPathNodes[i][0]<<",";
          cout<<"n2"<<currentPathNodes[i][1]<<endl;
        }
        */
        targetReached = true;
        encoders::go(0.0,baseSpeed,ps_right,ps_left,robot,timeStep);
        
      }
      
      if(not targetReached){
        turnToNextN(); //junction turning decisions
      }
      
     
      }
      if (known == true)
      {
      
        cout<<"known part"<<endl;
        //gen_paths(0,0,3,0,{});
        if (the_fix)
        { int curr_count = 0;
          for(int i = 0 ; i<67; i++)
          {
            
           if (path[i][0] ==  0 && path[i][1] ==0) 
           {
               curr_count = i;
               break;
           }
        
          }
          fill_up({current_node[0],current_node[1],path[curr_count-1][2]+1});
          the_fix = false;
        }
        int curr_loc_x = current_node[0]; 
        int curr_loc_y = current_node[1];
        cout<<"FffFfffffffffffffffffffffffffffffffffffffffffffffffffffffffff"<<curr_loc_x<<curr_loc_y<<endl;
        int curr_count = 0;
        for(int i = 0 ; i<67; i++)
            {
           //if (path[i][0] ==  curr_loc_x && path[i][1] ==curr_loc_y) 
           cout<<path[i][0]<<" "<<path[i][1]<<" "<<path[i][2]<<endl; 
        
            }
        cout<<curr_loc_x<<curr_loc_y<<curr_count<<endl;
        for(int i = 0 ; i<67; i++)
        {
           if (path[i][0] ==  curr_loc_x && path[i][1] ==curr_loc_y) 
           {
               curr_count = path[i][2];
               break;
           }
        
        }
        cout<<"FffFfffffffffffffffffffffffffffffffffffffffffffffffffffffffff"<<curr_count<<endl;
        curr_count -= 1;
        if(curr_count>0)
        {
        
        int next_x = 0;
        int next_y = 0;
        int x_side = 0;
        int y_side = 0;
        for(int i = 0 ; i<67; i++)
        {
           if (path[i][2] ==  curr_count && pow((pow((curr_loc_x - path[i][0]),2)+ pow((curr_loc_y - path[i][1]),2)),2) == 1 )
           {
               next_x = path[i][0];
               next_y = path[i][1];
               break;
           }
        
        }
        x_side = next_x - curr_loc_x;
        y_side = next_y - curr_loc_y;
       
        int dir_to_go = 0;
        if (x_side == 0)
        {
          if (y_side>0)
          {
            //encoders::go(0.035,baseSpeed,ps_right,ps_left,robot,timeStep)
            dir_to_go = 0;
          }
          else
          {
            dir_to_go = 2;
          }
        }
        else
        {
          if (x_side>0)
          {
            //encoders::go(0.035,baseSpeed,ps_right,ps_left,robot,timeStep)
            dir_to_go = 1;
          }
          else
          {
            dir_to_go = 3;
          }
        }
       int turn_side = dir_to_go - direction;
        cout<<"asddddddddddddddddddddd"<<dir_to_go<<" "<<direction<<" "<<turn_side<<endl;
       if ( turn_side== 0)
       {
         encoders::go(0.035,baseSpeed,ps_right,ps_left,robot,timeStep);
       }
       else if ( turn_side == -1 || turn_side == 3)
       {
         encoders::go(0.04,baseSpeed,ps_right,ps_left,robot,timeStep);
         turn_left();
       }
       else if ( turn_side ==1 || turn_side == -3)
       {
         encoders::go(0.04,baseSpeed,ps_right,ps_left,robot,timeStep);
         turn_right();
       }
       else
       {
         turn_left();
         turn_left();
       }
       //location_update();
       }
       else{
            if(current_node[0] == target_node[0] && current_node[1]==target_node[1])
            {
              stop();
               robot->step(10000);
               known = false;
            }
            else
            {
                
              int x_side = 0;
              int y_side = 0;
              x_side = target_node[0] - current_node[0];
              y_side = target_node[1] - current_node[1];
       
              int dir_to_go = 0;
              if (x_side == 0)
              {
                if (y_side>0)
                {
                  //encoders::go(0.035,baseSpeed,ps_right,ps_left,robot,timeStep)
                  dir_to_go = 0;
                }
              else
              {
                dir_to_go = 2;
              }
            }
            else
            {
              if (x_side>0)
              {
                //encoders::go(0.035,baseSpeed,ps_right,ps_left,robot,timeStep)
                dir_to_go = 1;
              }
              else
              {
                dir_to_go = 3;
              }
            }
           int turn_side = dir_to_go - direction;
            //cout<<"asddddddddddddddddddddd"<<dir_to_go<<" "<<direction<<" "<<turn_side<<endl;
           if ( turn_side== 0)
           {
             encoders::go(0.035,baseSpeed,ps_right,ps_left,robot,timeStep);
           }
           else if ( turn_side == -1 || turn_side == 3)
           {
             encoders::go(0.04,baseSpeed,ps_right,ps_left,robot,timeStep);
             turn_left();
           }
           else if ( turn_side ==1 || turn_side == -3)
           {
             encoders::go(0.04,baseSpeed,ps_right,ps_left,robot,timeStep);
             turn_right();
           }
           else
           {
             turn_left();
             turn_left();
           }
           //location_update();
           known = false;
           }
                 
         
       }
        
    
      }
      //cout<<box_in_front <<endl;
      location_update();
}
vector<string> if_picked_check(vector<string> content)
{
  vector<string> picked= {};
  for( int i =0 ; i<if_picked.size(); i++)
  {
    if (if_picked[i]==content[0])
    {
      picked.push_back(content[0]);
    }
    if (content.size()>1 && if_picked[i]==content[1])
    {
      picked.push_back(content[1]);
    }
  }
  
  return picked;
}

void maze_solve() /// Maze solve method
{
  
  int val = inverted_or_not(); /// Checking the type of line following needed
  //cout<<val<<endl;
  
  if (val == 0) ///White line following
  {
    miss_count = 0;
    //cout<<"white following"<<endl;
    string junc(white_line_follow(previous_error));///Getting juction conditions
    //cout<<junc<<endl;
    cout<<"dasdsadsaads"<<junc<<endl;
    if (junc != "i" && junc != "" && junc != "b" && junc != "s") // Checking if at a junction
    {
      
      junction_code(junc);///Juction code 
    }
    else if(junc == "b")
    {
      location_update();
      updateMap(2);
      updateNodes();
      //cout<<currently_carrying<<endl;
      //arm_methods::open_gripper(motors);
      vector<double> vals = lidar_methods::look_for_box(lidar,motors,robot);
      //cout<<vals[0]<<" "<<vals[1]<<endl;
      if (vals[1]<14)
      {
        double dis_to = 0.14-vals[1]/100;
        encoders::go(dis_to,-3,ps_right,ps_left,robot,timeStep);
      }
      if (vals.size()>0)
      {
       
       //cout<<out<<endl;
       if (current_status == "searching")
       {
         
         string left_over = "e";
         //cout<<"carrying "<<currently_carrying<<" "<<looking_for<<endl;
         if (currently_carrying == "w")
         {
           //cout<<"swap move 1"<<endl;
           arm_methods::swap_move_1(motors,robot);
           arm_methods::open_gripper(motors);
           robot->step(1000);
           arm_methods::arm_pos(0.12,1,motors);
           robot->step(1000);
           arm_methods::reset_arm(motors);
           robot->step(1000);
         }
         vector<string> out = check_box(vals);
         /*
         for(int i = 0; i< out.size(); i++)
         {
           cout<<out[i]<<endl;
         }
         cout<<"passed"<<out.size()<<endl;
         */
         if (out.size()==1)
         {
           //cout<<out[0]<<endl;
           vector<string> if_there = if_picked_check(out);
           if (if_there.size()==0)
           {
           if (out[0] != "w")
           {
           if (currently_carrying == "w")
           {
             //cout<<"swap"<<endl;//arm_methods::grab_8(motors);
             
             //arm_methods::arm_pos(0.12,1,motors);
             //robot->step(1000);
             //arm_methods::reset_arm(motors);
             //arm_methods::position_arm(vals,motors,robot);
             //robot->step(1000);
             //arm_methods::arm_pos(-0.01,1,motors);
             //robot->step(1000);
             arm_methods::grab_8(motors);
             robot->step(4500);
             arm_methods::arm_pos(0.12,1,motors);
             arm_methods::swap_move_2(motors);
             robot->step(3000);
             arm_methods::arm_pos(0.09,1,motors);///////////////
             robot->step(1000);
             arm_methods::open_gripper(motors);
             robot->step(100);
             arm_methods::swap_move_4(motors,robot);
             arm_methods::grab_8(motors);
             robot->step(4500);
             arm_methods::arm_pos(0.12,1,motors);
             robot->step(100);
             arm_methods::reset_arm(motors);
             robot->step(1000);
             arm_methods::arm_pos(-0.02,1,motors);///////////////
             robot->step(1500);
             arm_methods::open_gripper(motors);
             robot->step(1500);
             arm_methods::arm_pos(0.12,4,motors);
             robot->step(1500);
             arm_methods::swap_move_2(motors);
             robot->step(3000);
             arm_methods::arm_pos(0.08,1,motors);
             robot->step(1000);
             arm_methods::grab_8(motors);
             robot->step(4500);
             arm_methods::arm_pos(0.12,1,motors);
             robot->step(1000);
             arm_methods::reset_arm(motors);
             robot->step(1000);
             left_over = "w";
             currently_carrying = out[0];
             current_status= "returning";
           }
           else
           {
             //arm_methods::open_gripper(motors);
             robot->step(1000);
             arm_methods::grab_8(motors);
             robot->step(4500);
             currently_carrying = out[0];
             arm_methods::arm_pos(0.12,1,motors);
             robot->step(1000);
             arm_methods::reset_arm(motors);
             robot->step(500);
             
             
             //target_node = 0;
             //to_goal = pick_up_spot_start;
             
             
             current_status= "returning";
           }
           
           //updateMap(2);
           //updateNodes();
           //cout<<"lo00000000000000000000000000000000000000000000c"<<current_node[0]<<" "<<current_node[1]<<endl;
           if (currently_carrying == "b" && current_priority.size()>1)
               {
                 target_node = my_dock;
                 findShortestPath(current_node[0],current_node[1],target_node[0],(target_node[1]-1),0);
                 known = true;
                 the_fix = true;
               }
               else
               {
                 target_node = my_start;
                 findShortestPath(current_node[0],current_node[1],target_node[0]+1,target_node[1],0);
                 
                 known = true;
                 the_fix = true;
               }
           }
           else
           {
             if (looking_for != "w")
             {
               ///cout<<"remember white"<<endl;
               
               arm_methods::arm_pos(0.12,1,motors);
               robot->step(1000);
               arm_methods::reset_arm(motors);
               robot->step(1000);
               if (currently_carrying == "w")
               {
               
               arm_methods::swap_move_1(motors,robot);
               
               arm_methods::arm_pos(0.08,1,motors);
               robot->step(1000);
               arm_methods::grab_8(motors);
               robot->step(4500);
               arm_methods::arm_pos(0.12,1,motors);
               robot->step(1000);
               arm_methods::reset_arm(motors);
               robot->step(1000);
               currently_carrying = "w";
               }
               left_over = "w";
               ///set target to next suspected node / found node / go searching
             }
             else
             {
               arm_methods::grab_8(motors);
               robot->step(4500);
               arm_methods::arm_pos(0.12,1,motors);
               robot->step(1000);
               arm_methods::reset_arm(motors);
               robot->step(1000);
               looking_for = "rgb";
               current_status= "searching";///pref looking for inverted
               currently_carrying = "w";
               ////set goal to location
               ///set target to  inverted sq to unlock
             }
           }
           }
           else
           {
             arm_methods::arm_pos(0.12,1,motors);
             robot->step(1000);
             arm_methods::reset_arm(motors);
             robot->step(1000);
             if (currently_carrying == "w")
             {
             
             arm_methods::swap_move_1(motors,robot);
             
             arm_methods::arm_pos(0.08,1,motors);
             robot->step(1000);
             arm_methods::grab_8(motors);
             robot->step(4500);
             arm_methods::arm_pos(0.12,1,motors);
             robot->step(1000);
             arm_methods::reset_arm(motors);
             robot->step(1000);
           }
             //////node remember out[0]
             left_over = out[0];
             current_status= "searching";
             ///set target to next suspected node / found node / go searching
           }
         }
         else
         {
           vector<string> if_there = if_picked_check(out);
           if (if_there.size() <2)
           {
           //cout<<out[0]<<" "<<out[1]<<endl;
           if (currently_carrying == "w")
           {
            ///cout<<"swap"<<endl;//arm_methods::grab_8(motors);
            //cout<<if_there.size()<<endl;
            if ( (if_there.size()==0  || out[1] != if_there[0]) && out[1] != "w")
             {
               
               //cout<<"shouldnt be here"<<endl;
               arm_methods::grab_8(motors);
               robot->step(4500);
               arm_methods::arm_pos(0.12,1,motors);
               arm_methods::swap_move_2(motors);
               robot->step(3000);
               arm_methods::arm_pos(0.09,1,motors);///////////////
               robot->step(1000);
               arm_methods::open_gripper(motors);
               robot->step(100);
               arm_methods::swap_move_4(motors,robot);
               arm_methods::grab_8(motors);
               robot->step(4500);
               arm_methods::arm_pos(0.12,1,motors);
               robot->step(100);
               arm_methods::reset_arm(motors);
               robot->step(1000);
               arm_methods::arm_pos(0.03,1,motors);///////////////
               robot->step(1500);
               arm_methods::open_gripper(motors);
               robot->step(1500);
               arm_methods::arm_pos(0.12,4,motors);
               robot->step(1500);
               arm_methods::swap_move_2(motors);
               robot->step(3000);
               arm_methods::arm_pos(0.08,1,motors);
               robot->step(1000);
               arm_methods::grab_8(motors);
               robot->step(4500);
               arm_methods::arm_pos(0.12,1,motors);
               robot->step(1000);
               arm_methods::reset_arm(motors);
               robot->step(1000);
               
               currently_carrying = out[1];
               current_status= "returning";
               ///remember bottom
               left_over = out[0]+"w";
             }
             else
             {
               arm_methods::swap_move_1(motors,robot);
             
               arm_methods::arm_pos(0.08,1,motors);
               robot->step(1000);
               arm_methods::grab_8(motors);
               robot->step(4500);
               arm_methods::arm_pos(0.12,1,motors);
               robot->step(1000);
               arm_methods::reset_arm(motors);
               robot->step(1000);
               currently_carrying = "w";
               current_status= "searching";
               ///remember both
               left_over = out[0]+out[1];;
             }
           }
           else
           {
             
               if ( (if_there.size()>0 && out[1] == if_there[0] )||(out[1] =="w" && looking_for != "w"))
               {
                 
                 
                 arm_methods::grab_8(motors);
                 robot->step(4500);
                 arm_methods::swap_move_1(motors,robot);
                 ////////////////////////////1
                 //cout<<in_box_travel<<endl;
                 encoders::go(in_box_travel-0.008,-1,ps_right,ps_left,robot,timeStep);
                 arm_methods::open_gripper(motors);
                 robot->step(1000);
                 arm_methods::arm_pos(0.12,1,motors);
                 robot->step(1000);
                 arm_methods::reset_arm(motors);
                 arm_methods::position_arm(vals,motors,robot);
                 //robot->step(1000);
                 arm_methods::arm_pos(-0.02,1,motors);
                 robot->step(1000);
                 arm_methods::grab_8(motors);
                 robot->step(4500);
                 arm_methods::arm_pos(0.12,1,motors);
                 arm_methods::swap_move_2(motors);
                 robot->step(3000);
                 arm_methods::arm_pos(0.09,1,motors);///////////////
                 robot->step(1000);
                 arm_methods::open_gripper(motors);
                 robot->step(100);
                 arm_methods::swap_move_4(motors,robot);
                 //////////////////// 4
                 arm_methods::grab_8(motors);
                 robot->step(4500);
                 
                 arm_methods::arm_pos(0.12,1,motors);
                 robot->step(100);
                 arm_methods::reset_arm(motors);
                 robot->step(1000);
                 arm_methods::arm_pos(-0.01,1,motors);///////////////
                 robot->step(1500);
                 arm_methods::open_gripper(motors);
                 robot->step(1500);
                 arm_methods::arm_pos(0.12,4,motors);
                 robot->step(1500);
                 arm_methods::swap_move_2(motors);
                 robot->step(3000);
                 arm_methods::arm_pos(0.08,1,motors);
                 robot->step(1000);
                 arm_methods::grab_8(motors);
                 robot->step(4500);
                 arm_methods::arm_pos(0.12,1,motors);
                 robot->step(1000);
                 arm_methods::reset_arm(motors);
                 robot->step(1000);
                 currently_carrying = out[0];
                 current_status= "returning";
                 ///remember left one out[1]
                 left_over = out[1];
               }
               else
               {
                 //cout<<"herreer"<<endl;
                 arm_methods::grab_8(motors);
                 robot->step(4500);
                 arm_methods::arm_pos(0.12,1,motors);
                 robot->step(1500);
                 arm_methods::reset_arm(motors);
                 robot->step(1000);
                 currently_carrying = out[1];
                 left_over = out[0];
                 if (looking_for == "w")
                 {
                   current_status= "searching";////pref looking for inverted
                   looking_for == "rgb";
                 }
                 else
                 {
                   current_status= "returning";
                 }
               }
             //}
           }
           }
           else
           {
             arm_methods::arm_pos(0.12,1,motors);
             robot->step(1000);
             arm_methods::reset_arm(motors);
             robot->step(1000);
             if (currently_carrying == "w")
             {
             
             arm_methods::swap_move_1(motors,robot);
             
             arm_methods::arm_pos(0.08,1,motors);
             robot->step(1000);
             arm_methods::grab_8(motors);
             robot->step(4500);
             arm_methods::arm_pos(0.12,1,motors);
             robot->step(1000);
             arm_methods::reset_arm(motors);
             robot->step(1000);
           }
             //////node remember out[0],out[1]
             left_over = out[0]+out[1];;
             current_status= "searching";
             ///set target to next suspected node / found node / go searching
           }
           cout<<current_status<<endl;
           if (current_status == "returning")
           {
               
               if (currently_carrying == "b" && current_priority.size()>1)
               {
                 target_node = my_dock;
                 findShortestPath(current_node[0],current_node[1],target_node[0],(target_node[1]-1),0);
                 known = true;
                 the_fix = true;
               }
               else
               {
                 target_node = my_start;
                 findShortestPath(current_node[0],current_node[1],target_node[0]+1,target_node[1],0);
                 known = true;
                 the_fix = true;
               }
               
               
           }
           /*
           else
           {
             white goal;
           }
           */    
         }
         
         //arm_methods::grab_8(motors);
         //encoders::turnAng(-180,ps_left,ps_right,wheels,robot,timeStep);
         //updateMap(2);
         //updateNodes();
         pathNode * k = getNode(currentPoint[0],currentPoint[1]);
         k->blockContent = left_over; 
         if (robot->getName() == "robot_a")
          {
            
            cout<<k->blockContent<<endl;
          }
         //turn_left();
         //turn_left();
         box_in_front = lidar_methods::check_road(lidar,motors,robot);
         
         //stop_now = 1;
         //stop();
       }
       /*
       else if (current_status== "returning" && current_node[0] == 0)////Change to target node
       
       
         //cout<<"place"<<endl;
         //current_carrying = 
         
       */
       
       
      }
      else
      {
        cout<<"square"<<endl;
      }
    }
    
    else if(junc == "s")
    {
      if (current_priority.size()>1 &&currently_carrying == "b" )
      {
             arm_methods::arm_pos(-0.01,1,motors);///////////////
             robot->step(1500);
             arm_methods::open_gripper(motors);
             robot->step(1500);
             arm_methods::arm_pos(0.12,4,motors);
             robot->step(1500);
             to_goal = dock_to_goal;
             currently_carrying = "nothing";
             current_status = "searching";
             turn_left();
             turn_left();
             location_update();
             box_in_front = lidar_methods::check_road(lidar,motors,robot);
      }
      else if (current_priority.size()>2)
         {
           //swap
           if (box_count_at_start==0)
           {
             arm_methods::arm_pos(-0.01,1,motors);///////////////
             robot->step(1500);
             arm_methods::open_gripper(motors);
             robot->step(1500);
             arm_methods::arm_pos(0.12,4,motors);
             robot->step(1500);
             if (currently_carrying == "r")
             {
               current_priority.erase(current_priority.begin());
             }
            
            if_picked.push_back(currently_carrying);
            }
            else
            {
              vector<double> vals = lidar_methods::look_for_box(lidar,motors,robot);
              //cout<<vals[0]<<" "<<vals[1]<<endl;
              if (vals[1]<14)
              {
                double dis_to = 0.14-vals[1]/100;
                encoders::go(dis_to,-3,ps_right,ps_left,robot,timeStep);
              }
              
              arm_methods::swap_move_1(motors,robot);
              arm_methods::open_gripper(motors);
              robot->step(1000);
              arm_methods::arm_pos(0.12,1,motors);
              robot->step(1000);
              arm_methods::reset_arm(motors);
              robot->step(1000);
              
              vector<string> out = check_box(vals);
              
              arm_methods::grab_8(motors);
              robot->step(3000);
              arm_methods::arm_pos(0.12,1,motors);
              robot->step(1500);
              arm_methods::swap_move_2(motors);
              robot->step(3000);
              arm_methods::arm_pos(0.09,1,motors);
              robot->step(1000);
              arm_methods::open_gripper(motors);
              robot->step(1000);
              arm_methods::arm_pos(0.12,1,motors);
              robot->step(1000);
              arm_methods::reset_arm(motors);
              robot->step(1000);
              arm_methods::swap_move_4(motors,robot);
              arm_methods::arm_pos(0.08,1,motors);
              robot->step(1000);
              arm_methods::grab_8(motors);
              robot->step(3000);
              arm_methods::reset_arm(motors);
              robot->step(1500);
              
              arm_methods::arm_pos(-0.01,1,motors);
              robot->step(1500);
              arm_methods::open_gripper(motors);
              robot->step(1000);
              
              arm_methods::arm_pos(0.12,1,motors);
              robot->step(1000);
              arm_methods::swap_move_2(motors);
              robot->step(3000);
              arm_methods::arm_pos(0.08,1,motors);
              robot->step(1000);
              arm_methods::grab_8(motors);
              robot->step(3000);
              arm_methods::reset_arm(motors);
              robot->step(500);
              encoders::go(0.01,1,ps_right,ps_left,robot,timeStep);
              robot->step(1000);
              arm_methods::arm_pos(0.04,1,motors);
              robot->step(1500);
              arm_methods::open_gripper(motors);
              robot->step(1000);
              arm_methods::reset_arm(motors);
              arm_methods::arm_pos(0.12,1,motors);
              robot->step(1000);
              
              current_priority.erase(current_priority.begin());
              current_priority.erase(current_priority.begin());
            }
            updateMap(2);
            updateNodes();
            turn_left();
            turn_left();
            location_update();
              //if_picked.push_back({currently_carrying,"s"});
            box_in_front = lidar_methods::check_road(lidar,motors,robot);
            box_count_at_start += 1;
            currently_carrying = "nothing";
            current_status = "searching";
            //to_goal = pick_up_spot_start;
         }
         else
         {
            
            encoders::go(0.01*box_count_at_start,1,ps_right,ps_left,robot,timeStep);
            arm_methods::arm_pos((0.04*box_count_at_start),1,motors);
            robot->step(1000);
            arm_methods::open_gripper(motors);
            robot->step(1500);
            arm_methods::arm_pos(0.12,1,motors);
            robot->step(1000);
            to_goal = pick_up_spot_start;
            
            
            current_priority.erase(current_priority.begin());///
            
            
            turn_left();
            turn_left();
            location_update();
              //if_picked.push_back({currently_carrying,"s"});
            box_in_front = lidar_methods::check_road(lidar,motors,robot);
            box_count_at_start += 1;
              currently_carrying = "nothing";
            current_status = "searching";
            //to_goal = pick_up_spot_start;
            
         }
       ///////////Set target
       to_goal = pick_up_spot_start;
       
      
      /*
      turn_left();
      turn_left();
      //if_picked.push_back({currently_carrying,"s"});
      box_in_front = lidar_methods::check_road(lidar,motors,robot);
      box_count_at_start += 1;
      currently_carrying = "nothing";
      current_status = "searching";
      to_goal = pick_up_spot_start;
      */
    }
   
    
  }
  else if (val == 1)//Black line following
  {
    miss_count = 0;
    cout<<"black following"<<endl;
    string junc = black_line_follow(previous_error);
    if (junc != "i" && junc != "")
    {
      
      junction_code(junc,1);
     
    }
  }
  else{////Error handling
    //cout<<"non"<<endl;
    if (miss_count == 0)
    {
    encoders::go(0.05,baseSpeed,ps_right,ps_left,robot,timeStep);
    
    miss_count += 1;
    int val = inverted_or_not();
    if (val == 2)
    {
    vector<string> check = get_line_color();
    //cout<<"check"<<check[0]<<check[1]<<check[2]<<endl;
    if (check[0]=="0" && check[2]=="0")
    {
      //cout<<"white following non"<<endl;
      double line_loc = 0;
      int count = 0;
    
       
      for (int i = 0; i < 8 ; i++)
      {
      if (ir[i]->getValue()<400)
      {
        line_loc += i;
        count += 1;
      }
      }
      int point = line_loc/count;
      int error = point - 3.5;
      if (error<0)
      {
        //turnAng(-5);
        encoders::turnAng(-5,ps_left,ps_right,wheels,robot,timeStep);
      }
      else
      {
        //turnAng(5);
        encoders::turnAng(5,ps_left,ps_right,wheels,robot,timeStep);
      }
    }
    
    else
    {
      //cout<<"black following non"<<endl;
      string out = black_line_follow(previous_error);
      double line_loc = 0;
      int count = 0;
    
       
      for (int i = 0; i < 8 ; i++)
      {
      if (ir[i]->getValue()>400)
      {
        line_loc += i;
        count += 1;
      }
      }
      int point = line_loc/count;
      int error = point - 3.5;
      if (error<0)
      {
        
        encoders::turnAng(-5,ps_left,ps_right,wheels,robot,timeStep);
      }
      else
      {
        
        encoders::turnAng(5,ps_left,ps_right,wheels,robot,timeStep);
      }
    }
    }
    }
    else
    {
      stop();
      
    }
  }
}


void turn_left()
{
  //turnAng(-90);
  encoders::turnAng(-90,ps_left,ps_right,wheels,robot,timeStep);
  baseVal_ren = ps_right -> getValue();
  baseVal_len = ps_left -> getValue();
  direction -= 1;
  if (direction<0)
  {
    direction = 3;
  }
}
void turn_right()
{
  //turnAng(90);
  encoders::turnAng(90,ps_left,ps_right,wheels,robot,timeStep);
  baseVal_ren = ps_right -> getValue();
  baseVal_len = ps_left -> getValue();
  direction += 1;
  if (direction>3)
  {
    direction = 0;
  }
}


vector<string> check_box(vector<double> vals)
{
  arm_methods::open_gripper(motors);
  robot->step(1000);
  arm_methods::position_arm(vals,motors,robot);
  //cout<<gripper_ds[0]->getValue()<<" "<<gripper_ds[1]->getValue()<<endl;
  vector<double> dis_vals = {gripper_ds[0]->getValue(),gripper_ds[1]->getValue()};
  if (dis_vals[0]<200 || dis_vals[1]<200)
  {
  
    double dis_to_go = 0;
    if (dis_vals[0]<200 && dis_vals[1]<200)
    {
      dis_to_go = (dis_vals[1] + dis_vals[0])/2000;
    }
    else if(dis_vals[0]>200 && dis_vals[1]<200)
    {
      dis_to_go = dis_vals[1]/1000;
    }
    else if(dis_vals[0]<200 && dis_vals[1]>200)
    {
      dis_to_go = dis_vals[0]/1000;
    }
    //cout<<dis_to_go<<endl;
    if(dis_to_go>0.02)
    {
      dis_to_go -= 0.02;
      encoders::go(dis_to_go,1,ps_right,ps_left,robot,timeStep);
    }
    vector<string> objs = {get_color()};
    arm_methods::arm_pos(0.02,4,motors);
    robot->step(1000);
    bool grip = false;
    vector<double> dis_vals = {gripper_ds[0]->getValue(),gripper_ds[1]->getValue()};
    if (dis_vals[0]<200 || dis_vals[1]<200)
    {
      double dis_to_go = 0;
      if (dis_vals[0]<200 && dis_vals[1]<200)
      {
        dis_to_go = (dis_vals[1] + dis_vals[0])/2000;
      }
      else if(dis_vals[0]>200 && dis_vals[1]<200)
      {
        dis_to_go = dis_vals[1]/1000;
        grip = true;
      }
      else if(dis_vals[0]<200 && dis_vals[1]>200)
      {
        dis_to_go = dis_vals[0]/1000;
        grip = true;
      }
    //cout<<"top"<<dis_to_go<<endl;
      if(dis_to_go>0.02)
      {
        dis_to_go -= 0.02;
        encoders::go(dis_to_go,1,ps_right,ps_left,robot,timeStep);
      }
      if (grip == true)
      {
      arm_methods::grab_8(motors);
      robot->step(3000);
      arm_methods::open_gripper(motors);
      robot->step(1000);
      }
      in_box_travel = dis_to_go;
      objs.push_back(get_color());
    }
    else
    {
      arm_methods::arm_pos(-0.02,4,motors);
      return objs;
    }
  
   
    return objs;
  }
  else
  {
    cout<<"ERROR BOX NOT found"<<endl;
    
  }
  return {};
} 
/*
vector<int> path_gen(vector<int> path,vector<int> target_loc,vector<int>current_loc)
{
  if (target_loc == current_loc)
}
}
*/


void numgame(int x)
{
  if (x == 0)
  {
     return;
  }
  else
  {
    nums.push_back(x);
    numgame(x-1);
  }
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector<int> get_other_robot_loc()
{
  
  vector<int> loc;
   ifstream file (other_loc_file);
   string numm ;
   getline(file,numm);
   //int k = stoi(numm);
   stringstream ss(numm);
   
   cout<<"efe"<<endl;
   for (int i; ss >> i;) 
   {
        loc.push_back(i);
        //cout<<i<<endl;    
        if (ss.peek() == ',')
            ss.ignore();
    }
    
  cout<<loc.size()<<endl;
  file.close();
  
  return loc;
}
////////////get nodes called like  add_path_nodes({da,"sd",Da})
void add_path_nodes(vector<string>data)
{
  ofstream MyFile_A(my_nodes_found, ios::app);
  
  MyFile_A <<data[0]+","+data[1]+","+data[2]+","+data[3]+","+data[4]+","+data[5]+","+data[6]+"\n";
  MyFile_A.close();
}
////////////get nodes called like  vector<vector<int>>out =  get_added_nodes()
vector<vector<int>> get_added_nodes()
{
  vector<vector<int>> out = {};
  ifstream file (other_nodes_found);
  string line;
  int count = 0;
  int list_count = 0;
  while (getline(file, line))
  {
    stringstream ss(line);
   
   //cout<<"efe"<<endl;
   if (last_read==count)
   {
   out.push_back({});
   for (int i; ss >> i;) 
   {
        out[list_count].push_back(i);
        //cout<<i<<endl;    
        if (ss.peek() == ',')
            ss.ignore();
    }
    list_count += 1;
    last_read += 1;
    }
    count += 1;
  }
  file.close();
  return out;
}

void set_my_loc(vector<int>loc)
{
  ofstream MyFile_A(my_loc_file);
  
  MyFile_A <<to_string(loc[0])+","+to_string(loc[1])+"\n";
  MyFile_A.close();
}

vector<int> get_other_robot_target()
{
  
  vector<int> loc;
   ifstream file (other_target_file);
   string numm ;
   getline(file,numm);
   //int k = stoi(numm);
   stringstream ss(numm);
   
   //cout<<"efe"<<endl;
   for (int i; ss >> i;) 
   {
        loc.push_back(i);
        //cout<<i<<endl;    
        if (ss.peek() == ',')
            ss.ignore();
    }
    
  file.close();
  
  return loc;
}

void set_my_target(vector<int>loc)
{
  ofstream MyFile_A(my_target_file);
  
  MyFile_A <<to_string(targets[0][0])+","+to_string(targets[0][1]);
  MyFile_A.close();
}

int main(int argc, char **argv)
{

  robot = new Robot();
  timeStep = 32;
  
  
 
  ps_right = robot ->getPositionSensor("rightencorder");
  ps_left = robot ->getPositionSensor("leftencorder");
  
  ps_right -> enable(timeStep);
  ps_left -> enable(timeStep);

  lidar = robot ->getDistanceSensor("lidar");
  lidar->enable(timeStep);

  robot->step(100);
  for (int i = 0; i < 6 ; i++)
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

  for (int i = 0; i < 8 ; i++)
  {
    ir[i] = robot->getDistanceSensor(irnames[i]);
    ir[i]->enable(timeStep);

  }
  for (int i = 0; i < 4 ; i++)
  {
    
    camera[i]= robot->getCamera(camnames[i]);
    camera[i]->enable(timeStep);
  }
  for (int i = 0; i < 2 ; i++)
  {
    gripper_ds[i] = robot->getDistanceSensor(gripperdisnames[i]);
    gripper_ds[i]->enable(timeStep);

  }
  
  baseVal_ren = ps_right -> getValue();
  baseVal_len = ps_left -> getValue();
  //motors[0]->setVelocity(7.0);
 // motors[0]-> setPosition(0);
  lidar_methods::set_lidar_pos(0,7,motors);
  arm_methods::arm_pos(0.12,4,motors);
  vector<string> check = get_line_color();
  
  encoders::go(0.1,baseSpeed,ps_right,ps_left,robot,timeStep);
  to_goal = dock_to_goal;//pick_up_spot_start;//////////////////////////////
  
  if (robot->getName() == "robot_a")
  {
    current_node = {0,0};//{0,1};
    direction = 1;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    my_loc_file = "location_a.txt";
    other_loc_file = "location_b.txt";
    my_target_file = "target_a.txt";
    other_target_file = "target_b.txt";
    my_nodes_found = "robot_a.txt";
    other_nodes_found = "robot_b.txt";
    ofstream MyFile(my_nodes_found);
    MyFile.close();
    targets = {{1,1},{2,0},{3,1},{2,2},{3,2},{3,4},{2,4},{2,5},{5,5}};
    set_my_loc(current_node);//currentPoint);
    set_my_target(targets[0]);
    targets.erase(targets.begin());
    my_start = {0,0};
    my_dock = {1,1};
    other_start = {12,6};
    other_dock = {11,5};
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  else
  {
    current_node = {12,6};
    currentPoint = {12,6};
    direction = 3;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    my_loc_file = "location_b.txt";
    other_loc_file = "location_a.txt";
    my_target_file = "target_b.txt";
    other_target_file = "target_a.txt";
    my_nodes_found = "robot_b.txt";
    other_nodes_found = "robot_a.txt";
    targets = {{11,5},{10,6},{9,5},{8,4},{9,4},{9,2},{8,2},{9,1},{7,1}};
    set_my_loc(current_node);//currentPoint);
    set_my_target(targets[0]);
    targets.erase(targets.begin());
    other_start = {0,0};
    other_dock = {1,1};
    my_start = {12,6};
    my_dock = {11,5};
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  
  for(int i = 0 ; i <= 6; i++){
        for(int j = 0; j <= 12; j++){
         mapArr[j][i] = "  x  ";
         }
      }
  
  
  mapArr[0][0] = "00102";
  mapArr[12][6] = "10002";
  mapArr[1][1] = "00012";
  mapArr[11][5] = "01002";
  
  /*Total 5 digits
  first 4 connected path data : - path direction
  first - west
  last - south
  
  5th digit = junction type 
  0 = normal junction
  1 = inverted path
  2 = white patch
  */
  
  pathNode n1 = pathNode(currentPoint[0],currentPoint[1]);
  createdNodes[node_array_index] = n1;
  node_array_index++;
  
 /*
 vector<vector<double>> out = lidar_methods::scan_for_objects(lidar,motors,robot,current_node,direction);
 lidar_methods::set_lidar_pos(0,7,motors);
 for (int i = 0; i < out.size() ; i++)
  {
    cout<<robot->getName()<<" "<<out[i][0]<<" "<<out[i][1]<<endl;
  }
  */
  //robot->step(1000);
 /*
 arm_methods::arm_pos(0.07,4,motors);
  arm_methods::open_gripper(motors);
  vector<double> vals = lidar_methods::look_for_box(lidar,motors,robot);

  if (vals.size()>0)
  {
   pick_up_box(vals,6);
  }
  */
  /*
  arm_methods::arm_pos(0.07,7,motors);
  robot->step(100);
  cout<<lidar_methods::check_road(lidar,motors,robot)<<endl;
  */
  bool tr;
  
  
  
  while (robot->step(timeStep) != -1){
  
  
  if (box_count_at_start == 3)
  {
    break;
  }
  cout<<"location"<<current_node[0]<<" "<<current_node[1]<<endl;
  maze_solve();
 
  };
  ofstream MyFile(my_nodes_found);
  MyFile.close();
 
  
  delete robot;
  return 0;
}

