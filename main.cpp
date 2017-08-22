#include "MotionPlanner.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "MotionPlanner.h"
#include <bits/stdc++.h>

using namespace std;

vector<pos> v;
vector<pair> vertices;
path_params = Path_params();


void guiCallback(const krssg_ssl_msgs::Path::ConstPtr& point_SF){
	path_params.start.x = point_SF.s_x;
	path_params.start.y = point_SF.s_y;
	path_params.final.x = point_SF.f_x;
	path_params.final.x = point_SF.f_y;
	path_params.step_size = point_SF.step_size;
	path_params.bias_param = point_SF.bias_param;
	path_params.max_iteration = point_SF.max_iteration;
	path_params.selector = point_SF.selector;
}
void beliefStateCallback(const krssg_ssl_msgs::BeliefState::ConstPtr& msg){

  ROS_INFO("SHUBHAM,I AM IN CALLBACK!");
  pos p;
  v.clear();
  for(int i=0;i<msg->homePos.size();i++){
  	p.x=fabs(msg->homePos[i].x);
  	p.y=fabs(msg->homePos[i].y);
  	v.push_back(p);
  }

  for(int i=0;i<msg->awayPos.size();i++){
  	p.x=fabs(msg->awayPos[i].x);
  	p.y=fabs(msg->awayPos[i].y);
  	v.push_back(p);
  }

  Planning planning(v, v.size(), path_params);
  planning.planWithSimpleSetup();
  vector<pair> reals = planning.getReals();
  for(int i=0; i<getReals.size(); i++){
  	vertices[i].x = reals[i].x;
  	vertices[i].y = reals[i].y;
  }
}


int main(int argc, char **argv){

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  cout<<"HEY SHUBHAM!"<<endl;

  ros::Subscriber belief_sub = n.subscribe("/belief_state", 1000, beliefStateCallback);

  ros::Subscriber gui_sub = n.subscribe("gui_params", 1000, guiCallback);

  ros::Publisher path_pub = n.advertise<krssg_ssl_msgs::Path>("path_planner", 1000);

  ros::Rate loop_rate(10); //should be decided
  while(ros::ok){
  	krssg_ssl_msgs::Path path;
  	for(int i=0; i<vertices.size(); i++){
  		krssg_ssl_msgs::point vrtx;
  		vrtx.x = vertices[i].x;
  		vrtx.y = vertices[i].y; 
  		path.vertices.push_back(vrtx);
  	}
  	path_pub.publish(path);
  	ros::spinOnce();
  	loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
