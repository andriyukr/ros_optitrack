#include "fl/VelocityController.h"

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, odometry_msg->pose.pose.orientation.z;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;
}

void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
}

void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
    //cout << "[PID] velocity_d: " << velocity_d.transpose() << endl;
}

void dynamicReconfigureCallback(fuzzy_logic_controller::setControllerConfig &config, uint32_t level){
    kp = config.k_p;
    kd = config.k_d;
    if(config.i_internal){
        ki_external = 0;
        ki_internal = config.k_i;
    }
    else{
        ki_external = config.k_i;
        ki_internal = 0;
    }
    kv = config.k_v;

    int controller = config.controller;
    switch(controller){
        case 0:
            flcx = sflc;
            flcy = sflc;
            flcz = sflc;
            cout << "[VelocityController] SFLC used" << endl;
            break;
        case 1:
            flcx = snsflc;
            flcy = snsflc;
            flcz = snsflc;
            cout << "[VelocityController] Sta-NSFLC used" << endl;
            break;
        case 2:
            flcx = cnsflc;
            flcy = cnsflc;
            flcz = cnsflc;
            cout << "[VelocityController] Cen-NSFLC used" << endl;
            break;
    }
}

// Constructor
VelocityController::VelocityController(int argc, char** argv){
    ros::init(argc, argv, "fuzzy_velocity_controller");
    ros::NodeHandle node_handle;

    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback);
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback);
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback);

    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity", 1);

    flcx = new FuzzyLogicController("Fuzzy Logic Controller x");
    flcy = new FuzzyLogicController("Fuzzy Logic Controller y");
    flcz = new FuzzyLogicController("Fuzzy Logic Controller z");

    sflc = new SingletonFuzzyLogicController("Singleton Fuzzy Logic Controller");
    snsflc = new StandardNonSingletonFuzzyLogicController("Standard Non Singleton Fuzzy Logic Controller");
    cnsflc = new CentroidNonSingletonFuzzyLogicController("Centroid Non Singleton Fuzzy Logic Controller");

    pose << 0, 0, 0, 0;
    pose_d << 0, 0, 0, 0;
    velocity << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;

    error_i << 0, 0, 0, 0;
    error_old << 0, 0, 0, 0;

    if(argc > 1){
        kp = atof(argv[1]);
        ki_internal = atof(argv[2]);
        kd = atof(argv[3]);
        kv = atof(argv[4]);
    }
    else{
        kp = 2.0;
        ki_internal = 1.0;
        kd = 0.05;
        kv = 3;
    }
    ki_external = 0;
}

// Destructor
VelocityController::~VelocityController(){
    ros::shutdown();
    exit(0);
}

void VelocityController::run(){
    dynamic_reconfigure::Server<fuzzy_logic_controller::setControllerConfig> server;
    dynamic_reconfigure::Server<fuzzy_logic_controller::setControllerConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);
    while(ros::ok()){
      rate.sleep();
      ros::spinOnce();

      //ros::Time begin = ros::Time::now();

      if(pose_d(2) > 0){ // no command
          error = pose - pose_d;

          //error_d = (error - error_old) / dt;
          error_d = -(velocity_d - velocity);
          error_i += error * dt;
          error_old = error;

          geometry_msgs::Quaternion velocity_msg;
          velocity_msg.x = kv * flcx->getVelocity(kp * error(0), kd * error_d(0), ki_internal * error_i(0)) - ki_external * error_i(0);
          velocity_msg.y = kv * flcy->getVelocity(kp * error(1), kd * error_d(1), ki_internal * error_i(1)) - ki_external * error_i(1);
          velocity_msg.z = kv * flcz->getVelocity(kp * error(2), kd * error_d(2), ki_internal * error_i(2)) - ki_external * error_i(2);
          velocity_publisher.publish(velocity_msg);
      }

      //cout << "[VelocityController]: " << (ros::Time::now() - begin) << endl;
    }
}

int main(int argc, char** argv){
    cout << "Fuzzy Logic Velocity Controller running..." << endl;

    VelocityController* vc = new VelocityController(argc, argv);

    vc->run();
}
