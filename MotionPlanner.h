#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/est/EST.h>


#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerData.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

typedef struct{
    double x;
    double y;
}pos;

typedef struct{
    int x;
    int y;
}pair;

typedef struct{
    pair start;
    pair final;
    int step_size;
    int bias_param;
    int max_iteration;
    int selector;
}Path_params;

class Planning{
  public:
    Planning(vector<pos> &v, int n, Path_params path_params);
    void init(vector<pos> &v, int n, Path_params path_params);
    bool isStateValid(const ob::State *state);
    void planWithSimpleSetup();
    vector<pair> getReals();
  private:
    double* solvedPath;
    double* xc;
    double* yc;
    double* r;

    // Number of obstacles in space.
    int numObstacles;

    // Start position in space
    double xStart;
    double yStart;

    // Goal position in space
    double xGoal;
    double yGoal;

    // Boundaries of the space
    double xLeft;
    double xRight;
    double yTop;
    double yBottom;

     // Selector for path planner
    int selector;

    // all answer coordinates
    vector<pair> reals;

    // path planner's params
    int step_size;
    int bias_param;
    int max_iteration;
};
#endif