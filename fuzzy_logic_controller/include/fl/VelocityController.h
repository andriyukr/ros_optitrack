#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/QuaternionStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <dynamic_reconfigure/server.h>
#include <fuzzy_logic_controller/setControllerConfig.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

#include "fl/controllers/FuzzyLogicController.h"
#include "fl/controllers/SingletonFuzzyLogicController.h"
#include "fl/controllers/StandardNonSingletonFuzzyLogicController.h"
#include "fl/controllers/CentroidNonSingletonFuzzyLogicController.h"

using namespace geometry_msgs;
using namespace std;
using namespace fl;
using namespace ros;
using namespace Eigen;

// Subscribers
ros::Subscriber odometry_subscriber;
ros::Subscriber trajectory_subscriber;
ros::Subscriber trajectory_velocity_subscriber;

// Publishers
ros::Publisher velocity_publisher;

// Actual state
Vector4d pose;
Vector4d velocity;
Vector4d pose_d;
Vector4d velocity_d;

Vector4d error;
Vector4d error_old;
Vector4d error_d;
Vector4d error_i;

// Fuzzy Logic Controllers
FuzzyLogicController* flcx;
FuzzyLogicController* flcy;
FuzzyLogicController* flcz;

SingletonFuzzyLogicController* sflc;
StandardNonSingletonFuzzyLogicController* snsflc;
CentroidNonSingletonFuzzyLogicController* cnsflc;

// Gains
float kp;
float kd;
float ki_internal;
float ki_external;
float kv;

class VelocityController{
        public:
          VelocityController(int, char**);
          ~VelocityController();
          void run();
};
