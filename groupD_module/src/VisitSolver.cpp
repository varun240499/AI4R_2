    /*
     <one line to give the program's name and a brief idea of what it does.>
     Copyright (C) 2015  <copyright holder> <email>
     
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
     */

#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "cstdlib"
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <random>
#include "armadillo"
#include <initializer_list>

using namespace std;
using namespace arma;

    //map <string, vector<double> > region_mapping;

extern "C" ExternalSolver* create_object(){
  return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver){
  delete externalSolver;
}

VisitSolver::VisitSolver(){
}

VisitSolver::~VisitSolver(){
}

void VisitSolver::loadSolver(string *parameters, int n){
  starting_position = "r0";
  string Paramers = parameters[0];

  char const *x[]={"dummy"};
  char const *y[]={"act-cost","triggered"};
  parseParameters(Paramers);
  affected = list<string>(x,x+1);
  dependencies = list<string>(y,y+2);

  string waypoint_file = "/root/popf-tif/domains/groupD_domain/waypoint.txt";
  parseWaypoint(waypoint_file);

  string landmark_file = "/root/popf-tif/domains/groupD_domain/landmark.txt";
  parseLandmark(landmark_file);

}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic){

  map<string, double> toReturn;
  map<string, double>::iterator iSIt = initialState.begin();
  map<string, double>::iterator isEnd = initialState.end();
  map<string, double> trigger;
  
  double dummy = 0;
  double act_cost = 0;

  for(;iSIt!=isEnd;++iSIt){

    string parameter = iSIt->first;
    string function = iSIt->first;
    double value = iSIt->second;

    function.erase(0,1);
    function.erase(function.length()-1,function.length());
    int n=function.find(" ");

    if(n!=-1){
      string arg=function;
      string tmp = function.substr(n+1,5);

      function.erase(n,function.length()-1);
      arg.erase(0,n+1);
      if(function=="triggered"){
        trigger[arg] = value>0?1:0;
        if (value>0){

      		string from = tmp.substr(0,2);   // from and to are regions, need to extract wps (poses)
      		string to = tmp.substr(3,2);
      		
			euclidean_dis(from, to);
       }
      }
    }

    else{
      if(function=="dummy"){
        dummy = value;
      }
      else if(function=="act-cost"){
        act_cost = value;
      } 
    }
}

  double results = calculateExtern(dummy, act_cost);
  toReturn["(dummy)"] = results;

  return toReturn;
}

list<string> VisitSolver::getParameters(){

  return affected;
}

list<string> VisitSolver::getDependencies(){

  return dependencies;
}


void VisitSolver::parseParameters(string parameters){

  int curr, next;
  string line;
  ifstream parametersFile(parameters.c_str());
  if (parametersFile.is_open()){
    while (getline(parametersFile,line)){
     curr=line.find(" ");
     string region_name = line.substr(0,curr).c_str();
     curr=curr+1;
     while(true ){
      next=line.find(" ",curr);
      region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());
      if (next ==-1)
       break;
     curr=next+1;

     }                
   }  
  }
}

double VisitSolver::calculateExtern(double external, double total_cost){
  //float random1 = static_cast <float> (rand())/static_cast <float>(RAND_MAX);
  double cost = dist;//random1;
  cout << "distance : " << cost << endl;

  return cost;
}

void VisitSolver::parseWaypoint(string waypoint_file){

       int curr, next;
       string line;
       double pose1, pose2, pose3;
       ifstream parametersFile(waypoint_file);
       if (parametersFile.is_open()){
        while (getline(parametersFile,line)){
         curr=line.find("[");
         string waypoint_name = line.substr(0,curr).c_str();

         curr=curr+1;
         next=line.find(",",curr);

         pose1 = (double)atof(line.substr(curr,next-curr).c_str());
         curr=next+1; next=line.find(",",curr);

         pose2 = (double)atof(line.substr(curr,next-curr).c_str());
         curr=next+1; next=line.find("]",curr);

         pose3 = (double)atof(line.substr(curr,next-curr).c_str());

         waypoint[waypoint_name] = vector<double> {pose1, pose2, pose3};
       }
     }
}

void VisitSolver::parseLandmark(string landmark_file){

     int curr, next;
     string line;
     double pose1, pose2, pose3;
     ifstream parametersFile(landmark_file);
     if (parametersFile.is_open()){
      while (getline(parametersFile,line)){
       curr=line.find("[");
       string landmark_name = line.substr(0,curr).c_str();
       
       curr=curr+1;
       next=line.find(",",curr);

       pose1 = (double)atof(line.substr(curr,next-curr).c_str());
       curr=next+1; next=line.find(",",curr);

       pose2 = (double)atof(line.substr(curr,next-curr).c_str());
       curr=next+1; next=line.find("]",curr);

       pose3 = (double)atof(line.substr(curr,next-curr).c_str());

       landmark[landmark_name] = vector<double> {pose1, pose2, pose3};
     }
   }
   
}

void VisitSolver::euclidean_dis( string from, string to){
    
    string wp[5] = {"wp0","wp1","wp2","wp3","wp4"};
    map <string, string> reg_wp;
    reg_wp["r0"] = wp[0];
    reg_wp["r1"] = wp[1];
    reg_wp["r2"] = wp[2];
    reg_wp["r3"] = wp[3];
    reg_wp["r4"] = wp[4];

    double wp_x1 = waypoint[reg_wp[from]].at(0);
    double wp_y1 = waypoint[reg_wp[from]].at(1);
    
    double wp_x2 = waypoint[reg_wp[to]].at(0);
    double wp_y2 = waypoint[reg_wp[to]].at(1);
    
    dist = sqrt(pow((wp_x2-wp_x1),2) + pow((wp_y2-wp_y1),2));
}

