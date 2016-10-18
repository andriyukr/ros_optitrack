#include "controllers/logger.h"

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, odometry_msg->pose.pose.orientation.z;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z;
    angular_velocity << odometry_msg->twist.twist.angular.x, odometry_msg->twist.twist.angular.y, odometry_msg->twist.twist.angular.z;
}

void trajectoryCallback(const geometry_msgs::QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
}

void trueOdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    pose_r << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, odometry_msg->pose.pose.orientation.z;
}

void velocityCallback(const geometry_msgs::Quaternion& velocity_msg){
    velocity_d << velocity_msg.x, velocity_msg.y, velocity_msg.z, velocity_msg.w;
}

void motorSpeedCallback(const geometry_msgs::QuaternionStamped& motor_speed_msg){
    omega << motor_speed_msg.quaternion.x, motor_speed_msg.quaternion.y, motor_speed_msg.quaternion.z, motor_speed_msg.quaternion.w;
}

// Constructor
Logger::Logger(int argc, char** argv){
    ros::init(argc, argv, "Logger");
    ros::NodeHandle node_handle;

    char file[100];
    strcpy(file, "/home/ste/Results/");
    strcat(file, argv[1]);
    strcat(file, ".txt");
    results.open(file);

    //cout << "*****[Logger]: file = " << file << endl;

    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback);
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback);
    true_odometry_subscriber = node_handle.subscribe("/uav/true_odometry", 1, trueOdometryCallback);
    velocity_subscriber = node_handle.subscribe("/uav/command_velocity", 1, velocityCallback);
    motor_speed_subscriber = node_handle.subscribe("/uav/command/motor_speed", 1, motorSpeedCallback);
}

// Destructor
Logger::~Logger(){
    ros::shutdown();
    exit(0);
}

void Logger::run(){
    ros::Rate rate(100);
    while(ros::ok()){
      rate.sleep();
      ros::spinOnce();

      results << ros::Time::now() << ", "                                                                   // 1
              << pose(0) << ", " << pose(1) << ", " << pose(2) << ", " << pose(3) << ", "                   // 2, 3, 4, 5
              << pose_d(0) << ", " << pose_d(1) << ", " << pose_d(2) << ", " << pose_d(3) << ", "           // 6, 7, 8, 9
              << pose_r(0) << ", " << pose_r(1) << ", " << pose_r(2) << ", " << pose_r(3) << ", "           // 10, 11, 12, 13
              << velocity(0) << ", " << velocity(1) << ", " << velocity(2) << ", "                          // 14, 15, 16
              << velocity_d(0) << ", " << velocity_d(1) << ", " << velocity_d(2) << ", "                    // 17, 18, 19
              << angular_velocity(0) << ", " << angular_velocity(1) << ", " << angular_velocity(2) << ", "  // 20, 21, 22
              << omega(0) << ", " << omega(1) << ", " << omega(2) << ", " << omega(3) << endl;              // 22, 23, 24, 25
    }
}

int main(int argc, char** argv){
    cout << "[logger] Logger is running..." << endl;

    Logger* logger = new Logger(argc, argv);

    logger->run();
}
