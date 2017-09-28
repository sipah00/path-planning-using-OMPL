#include "ros/ros.h"
#include "std_msgs/String.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include <iostream>
#include "MotionPlanner.h"
#include <bits/stdc++.h>

using namespace std;

std::vector<pos> v;
//Planning *planning;

void Callback(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
  ROS_INFO("SHUBHAM,I AM IN CALLBACK!");

  pos p;
  v.clear();
  for(int i=0;i<msg->homePos.size();i++){
  	p.x=fabs(msg->homePos[i].x+3300)/8.0;
  	p.y=fabs(msg->homePos[i].y+2200)/8.0;
  	//cout<<p.x<<" "<<p.y<<endl;
  	v.push_back(p);
  }

  for(int i=0;i<msg->awayPos.size();i++){
  	p.x=fabs(msg->awayPos[i].x+3300)/8.0;
  	p.y=fabs(msg->awayPos[i].y+2200)/8.0;
  	v.push_back(p);
  }

  Planning planning(v,v.size());
  planning.planWithSimpleSetup();
  planning.drawPath();
  planning.output();

  planning.planSimple();
  planning.plan(0, 0, 800, 800);
  planning.recordSolution();
  planning.drw();
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  cout<<"HEY SHUBHAM!"<<endl;

  /*pos p;
  time_t sec;
  time(&sec);
  srand((unsigned int)sec);*/

  ros::Subscriber sub = n.subscribe("/belief_state", 1000, Callback);
  /*for(int i=0;i<12;i++){
  	p.x=400.0;//rand()%800;
  	p.y=400.0;//rand()%800;
  	v.push_back(p);
  }

  while(1){
  Planning planning(v,v.size());
  planning.planWithSimpleSetup();
  planning.drawPath();
  planning.output();
  }*/

  ros::spin();

  return 0;
}
