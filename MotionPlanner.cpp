#include "MotionPlanner.h"

using namespace std;

Planning::Planning(vector<pos> &v, int n, Path_params path_params){
  init(v,n,path_params);
}

void Planning::init(vector<pos> &v, int n, Path_params path_params){
  // Start position in space
  xStart = path_params.start.x; 
  yStart = path_params.start.y;

  // Goal position in space
  xGoal = path_params.final.x;
  yGoal = path_params.final.y;

  // Max. distance toward each sampled position we should grow our tree
  step_size = path_params.step_size;
  bias_param = path_params.bias_param;
  max_iteration = path_params.max_iteration;

  // Boundaries of the space 
  xLeft = 0.0;  //dummy values
  xRight = 800.0;
  yTop = 800.0;
  yBottom = 0.0;

  // Choose your path planner
  selector = path_params.selector;

  // Number of Obstacles including our bots lol!
  numObstacles = n;
  xc = new double[numObstacles];
  yc = new double[numObstacles];

  for(int i = 0; i<n; i++){
    xc[i] = v[i].x;
    yc[i] = v[i].y;
  }
}

bool Planning::isStateValid(const ob::State *state){
  const ob::SE2StateSpace::StateType *state_2d= state->as<ob::SE2StateSpace::StateType>();
  const double &x(state_2d->getX()), &y(state_2d->getY());

  for (int i = 0; i < numObstacles; ++i){
    if (sqrt(pow((xc[i]-x),2)+pow((yc[i]-y),2)) <= BOT_RADIUS){
      return false;
    }
  }
  return true;
}

void Planning::getReals(){
  return reals;
}

void Planning::planWithSimpleSetup(){
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
  }else if(selector == 10){
    ob::PlannerPtr planner(new og::InformedRRTstar(ss.getSpaceInformation()));
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

    // const base::StateSpace *space(si_->getStateSpace().get());
    // std::vector<double> reals;
    // for(auto state:states_){
    //   space->copyToReals(reals,state);
    //   solvedPath.push_back(reals);
    // }
    og::PathGeometric &p = ss.getSolutionPath();
    p.interpolate();
    reals.clear();
    for(std::size_t i = 0; i<p.getStateCount(); i++){
      int x = min(xRight, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->values[0]);
      int y = min(yBottom, (int)p.getState(i)->as<ob::SE2StateSpace::StateType>()->values[1]);
      pair pos;
      pos.x = x, pos.y = y;
      reals.push_back(pos);
    }
    //Print the solution path to a file
    //std::ofstream ofs("path.dat");
    //ss.getSolutionPath().printAsMatrix(ofs);
  }
  else
    cout << "No solution found" << endl;
}
