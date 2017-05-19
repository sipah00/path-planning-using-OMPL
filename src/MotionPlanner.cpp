#include "MotionPlanner.h"

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

#define radius 10

using namespace cv;
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef struct 
{
  double x;
  double y;
}point;


Mat img(800,800,CV_8UC3,Scalar(0,255,0));

std::vector<point> vect;

Planning::Planning(vector<pos> &v,int n){
  init(v,n);
  CreateCircle();
}


void Planning::init(vector<pos> &v,int n)
{

  xStart=0.0;
  yStart=0.0;
    // Goal position in space
  xGoal=800.0;
  yGoal=500.0;
    // Max. distance toward each sampled position we
    // should grow our tree
  stepSize=0.5;
    // Boundaries of the space
  xLeft=0.0;
  xRight=800.0;
  yTop=800.0;
  yBottom=0.0;
  selector=3;


  numObstacles = n;
  xc = new double[numObstacles];
  yc = new double[numObstacles];

  for(int i=0;i<n;i++){
    xc[i] = v[i].x;
    yc[i] = v[i].y;
  }
  
  for(int i = 0; i < numObstacles; ++i){
    printf("Circle %d : [x,y]--[%5.2lf, %5.2lf] radius--[%5.2lf]\n", i+1, xc[i], yc[i],30.0);
  }

  printf("\nStart [%5.2lf, %5.2lf]\n", xStart, yStart);
  printf("End [%5.2lf, %5.2lf]\n\n", xGoal, yGoal);

}


void Planning::CreateCircle(){
    for(int i=0;i<numObstacles;i++){
      circle(img,Point(xc[i],yc[i]),10.0,Scalar(250,128,114),CV_FILLED,8,0);
    }
}


bool Planning::isStateValid(const ob::State *state)
{
  const ob::SE2StateSpace::StateType *state_2d= state->as<ob::SE2StateSpace::StateType>();
  const double &x(state_2d->getX()), &y(state_2d->getY());

  for (int i = 0; i < numObstacles; ++i){
    if (sqrt(pow((xc[i]-x),2)+pow((yc[i]-y),2))<=10.0){
      return false;
    }
  }

  return true;
}


void Planning::planWithSimpleSetup()
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0,xLeft);
  bounds.setHigh(0,xRight);
  bounds.setLow(1,yBottom);
  bounds.setHigh(1,yTop);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&Planning::isStateValid, this, _1));

  // Setup Start and Goal
  ob::ScopedState<ob::SE2StateSpace> start(space);
  start->setXY(xStart,yStart);
  //cout << "start: "; start.print(cout);

  ob::ScopedState<ob::SE2StateSpace> goal(space);
  goal->setXY(xGoal,yGoal);
  //cout << "goal: "; goal.print(cout);

  ss.setStartAndGoalStates(start, goal);

  if(selector == 1){
    ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 2){
    ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 3){
    ob::PlannerPtr planner(new og::RRTConnect(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 4){
    ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 5){
    ob::PlannerPtr planner(new og::LBTRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 6){
    ob::PlannerPtr planner(new og::LazyRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 7){
    ob::PlannerPtr planner(new og::TRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 8){
    ob::PlannerPtr planner(new og::pRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 9){
    ob::PlannerPtr planner(new og::EST(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }

  cout << "----------------" << endl;

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(5);

  // If we have a solution,
  if (solved)
  {
    ss.getSolutionPath();

    // Simplify the solution
    ss.simplifySolution();
    cout << "----------------" << endl;
    cout << "Found solution:" << endl;


    //og::PathGeometric p(ss.getSolutionPath());
    /*og::PathGeometric &p = (ss.getSolutionPath());
    //ob::SE2StateSpace::StateType *p=ss.getSolutionPath();
    //p = new og::PathGeometric((og::PathGeometric &)ss.getSolutionPath);
    //cout<<"prabh"<<endl;

        p.interpolate();
        //cout<<"prabh"<<endl;
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
        {

            cout<<p.getStateCount()<<"ckn"<<endl;
            //const double &x(p->getX()), &y(p->getY())
            const int w = std::min(800,(int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h = std::min(800,(int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            point temp;
            temp.x = h;//min(800,(int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            temp.y = w;//min(800, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            cout<<w<<" "<<h<<endl;
            vect.push_back(temp);
        }*/
        //cout<<"prabh"<<endl;

    //Print the solution path to a file
    std::ofstream ofs("path.dat");
    ss.getSolutionPath().printAsMatrix(ofs);
    /*og::PathGeometric &p=ss.getSolutionPath();
    p.interpolate();
    cout<<" hwy"<<endl;
    for(size_t i=0;i<p.getStateCount();i++){
      point temp;
      temp.x = min(800,(int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
      temp.y = min(800, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
      vect.push_back(temp);
      cout<<" hwy"<<endl;
    }*/
  }
  else
    cout << "No solution found" << endl;
}
 
void Planning::drawPath(){

  double temp;
  point p;
  vector<point> vec;
   std::ifstream input("path.dat");
   if (!input) {
    std::cerr << "error file" << std::endl;
    std::exit(1);
  }

  while(!input.eof()){
    input>>p.x>>p.y>>temp;
    vec.push_back(p);
  }

  for(int i=0;i<vec.size()-1;i++){
    line(img,Point(vec[i].x,vec[i].y),Point(vec[i+1].x,vec[i+1].y),Scalar(32,178,170),2,8,0);
  }

  /*for(int i=0;i<vect.size()-1;i++){
    line(img,Point(vect[i].x,vect[i].y),Point(vect[i+1].x,vect[i+1].y),Scalar(32,178,170),2,8,0);
   }*/

}

void Planning::output(){
  circle(img,Point(xStart,yStart),5,Scalar(250,0,0),CV_FILLED,8,0);
  circle(img,Point(xGoal,yGoal),5,Scalar(250,0,0),CV_FILLED,8,0);
  namedWindow( "OMPL", WINDOW_AUTOSIZE );
  imshow("OMPL",img);
  waitKey(1);
  img=Scalar(0,255,0);
}
