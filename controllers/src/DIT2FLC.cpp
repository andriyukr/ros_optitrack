#include "controllers/DIT2FLC.h"

void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    pose << odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z, odometry_msg->pose.pose.orientation.z;
    velocity << odometry_msg->twist.twist.linear.x, odometry_msg->twist.twist.linear.y, odometry_msg->twist.twist.linear.z, odometry_msg->twist.twist.angular.z;
    //cout << "[DIT2FLC] position: " << pose.transpose() << endl;
}

void trajectoryCallback(const QuaternionStamped& trajectory_msg){
    pose_d << trajectory_msg.quaternion.x, trajectory_msg.quaternion.y, trajectory_msg.quaternion.z, trajectory_msg.quaternion.w;
    //cout << "[DIT2FLC] position: " << pose_d.transpose() << endl;
}

void trajectoryVelocityCallback(const QuaternionStamped& velocity_msg){
    velocity_d << velocity_msg.quaternion.x, velocity_msg.quaternion.y, velocity_msg.quaternion.z, velocity_msg.quaternion.w;
    //cout << "[DI-IT2-FLC] velocity_d: " << velocity_d.transpose() << endl;
}

void dynamicReconfigureCallback(controllers::setDIT2FLCConfig &config, uint32_t level){
    k_p = config.k_p;
    k_i = config.k_i;
    k_d = config.k_d;

    k_a = config.k_a;
    k_b = config.k_b;

    alpha_p = config.alpha_p;
    alpha_d = config.alpha_d;
}

// Constructor
DIT2FLC::DIT2FLC(int argc, char** argv){
    ros::init(argc, argv, "DI_T2_FPID");
    ros::NodeHandle node_handle;

    odometry_subscriber = node_handle.subscribe("/uav/odometry", 1, odometryCallback);
    trajectory_subscriber = node_handle.subscribe("/uav/trajectory", 1, trajectoryCallback);
    trajectory_velocity_subscriber = node_handle.subscribe("/uav/trajectory_velocity", 1, trajectoryVelocityCallback);

    velocity_publisher = node_handle.advertise<geometry_msgs::Quaternion>("/uav/command_velocity", 1);

    pose << 0, 0, 0, 0;
    pose_d << 0, 0, 0, 0;
    velocity << 0, 0, 0, 0;
    velocity_d << 0, 0, 0, 0;

    error_i << 0, 0, 0, 0;
    error_old << 0, 0, 0, 0;
    phi_i << 0, 0, 0, 0;

    if(argc > 1){
        k_p = atof(argv[1]);
        k_i = atof(argv[2]);
        k_d = atof(argv[3]);
        k_a = atof(argv[4]);
        k_b = atof(argv[5]);
        alpha_p = atof(argv[6]);
        alpha_d = atof(argv[7]);
    }
    else{
        k_p = 1.0;
        k_i = 0.1;
        k_d = 0.004;
        k_a = 0.077;
        k_b = 7.336;
        alpha_p = 0.2;
        alpha_d = 0.6;
    }

    k_e = (double)1 / E_MAX;

    return;
}

// Destructor
DIT2FLC::~DIT2FLC(){
    ros::shutdown();
    exit(0);
}

void DIT2FLC::run(){
    dynamic_reconfigure::Server<controllers::setDIT2FLCConfig> server;
    dynamic_reconfigure::Server<controllers::setDIT2FLCConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate(100);
    while(ros::ok()){
      rate.sleep();
      ros::spinOnce();

      //ros::Time begin = ros::Time::now();

      if(pose_d(2) > 0){ // no command

          //cout << "[DIT2FLC]: pose_d = " << pose_d.transpose() << endl;
          //cout << "[DIT2FLC]: pose = " << pose.transpose() << endl;

          error = pose_d - pose;
          error_i += error * dt;
          error_d = velocity_d - velocity;
          //error_d = (error - error_old) / dt;
          error_old = error;

          //cout << "[DIT2FLC]: error = " << error.transpose() << endl;

          sigma_p << max(-1.0, min(1.0, k_p * error(0))), max(-1.0, min(1.0, k_p * error(1))), max(-1.0, min(1.0, k_p * error(2))), max(-1.0, min(1.0, k_p * error(3)));
          sigma_d << max(-1.0, min(1.0, k_d * error_d(0))), max(-1.0, min(1.0, k_d * error_d(1))), max(-1.0, min(1.0, k_d * error_d(2))), max(-1.0, min(1.0, k_d * error_d(3)));

          //cout << "[DIT2FLC]: sigma_p = " << sigma_p.transpose() << endl;
          //cout << "[DIT2FLC]: sigma_d = " << sigma_d.transpose() << endl;

          phi << evalt2(sigma_p(0), sigma_d(0)), evalt2(sigma_p(1), sigma_d(1)), evalt2(sigma_p(2), sigma_d(2)), evalt2(sigma_p(3), sigma_d(3));

          //cout << "[DIT2FLC]: phi = " << phi.transpose() << endl;

          phi_i += phi * dt;

          geometry_msgs::Quaternion velocity_msg;
          velocity_msg.x = k_a * phi(0) + k_b * phi_i(0) + k_i * error_i(0);
          velocity_msg.y = k_a * phi(1) + k_b * phi_i(1) + k_i * error_i(1);
          velocity_msg.z = k_a * phi(2) + k_b * phi_i(2) + k_i * error_i(2);
          velocity_msg.w = 0; //k_a * phi(3) + k_b * phi_i(3) + k_i * error_i(3);
          velocity_publisher.publish(velocity_msg);
      }

      //cout << "[DIT2FLC]: u = " << (k_a * phi + k_b * phi_i + k_i * error_i).transpose() << endl;
      //cout << "[DIT2FLC]: *****" << endl;
    }
}

double DIT2FLC::evalt2(double ep, double ed){
    MatrixXi rules(9, 3);
    rules << 1, 1, 1,
             1, 2, 2,
             1, 3, 3,
             2, 1, 2,
             2, 2, 3,
             2, 3, 4,
             3, 1, 3,
             3, 2, 4,
             3, 3, 5;

    MatrixXd input(12, 4);
    input << -3.5, -1, 0, 1,
              -3.5, -1, 0, 0.8,
              -1, 0, 1, 1,
              -1, 0, 1, 0.2,
              0, 1, 3.5, 1,
              0, 1, 3.5, 0.8,

              -3.5, -1, 0, 1,
              -3.5, -1, 0, 0.4,
              -1, 0, 1, 1,
              -1, 0, 1, 0.6,
              0, 1, 3.5, 1,
              0, 1, 3.5, 0.4;

    MatrixXd output(5, 1);
    output << -1,
              -0.8,
              0,
              0.8,
              1;

    int NofRule = 9;
    int nInput = 2;
    double y = 0;

    MatrixXd F = MatrixXd::Zero(NofRule, nInput);
    MatrixXd C = MatrixXd::Zero(NofRule, 2);
    Vector2d x;
    x << ep, ed;

    for(int n = 0; n < NofRule; ++n){
        double f1U = 1;
        double f1L = 1;

        for(int i = 0; i < nInput; ++i){
            // Calculate Lower firing
            Vector4d UpperParams, LowerParams;

            UpperParams = input.row(6 * i + 2 * rules(n, i) - 2);
            LowerParams = input.row(6 * i + 2 * rules(n, i) - 1);

            double MemberUpper = UpperParams(3) * trimf(x(i), UpperParams(0), UpperParams(1), UpperParams(2));
            double MemberLower = LowerParams(3) * trimf(x(i), UpperParams(0), UpperParams(1), UpperParams(2));

            f1U = f1U * MemberUpper;
            f1L = f1L * MemberLower;
        }

        F.row(n) << f1L, f1U;

        double outMFPar = output(rules(n, 2) - 1);
        C.row(n) << outMFPar, outMFPar;
    }

    //cout << "*** F:\n" << F << endl;
    //cout << "*** C:\n" << C << endl;

    // Calculate Output
    return km(F, C);
}

/* http://www.mathworks.com/help/fuzzy/trimf.html */
double DIT2FLC::trimf(double x, double a, double b, double c){
    return max(min((x - a) / (b - a), (c - x) / (c - b)), 0.0);
}

double DIT2FLC::km(MatrixXd F, MatrixXd Y){
    double yLeft = 0, yRight = 0;

    /* KM Algorithm for Computing Y Left */

    // a) Sort Y matrix
    VectorXd lowerY;
    lowerY = Y.col(0);

    //cout << "lowerY:\n" << lowerY.transpose() << endl;
    //cout << "F:\n" << F.transpose() << endl;

    MatrixXd ind;
    ind = sort(lowerY, F);

    MatrixXd sortedF;
    lowerY = ind.col(0);
    sortedF = ind.rightCols(2);

    //cout << "sorted_lowerY:\n" << lowerY.transpose() << endl;
    //cout << "sortedF:\n" << sortedF.transpose() << endl;

    // b) Initialize fn by setting and computing
    VectorXd FOrt;
    FOrt = sum(F) / 2;

    //cout << "FOrt:\n" << FOrt << endl;

    bool isZero = (sum(FOrt)(0) == 0);
    int nRules = 9;

    //cout << "isZero:\n" << isZero << endl;

    double yn;
    if(isZero)
        yn = 0;
    else
        yn = sum(lowerY.cwiseProduct(FOrt))(0) / sum(FOrt)(0);

    //cout << "yn:\n" << yn << endl;

    while(1){
        // c) Find switch point k (1 <= k <= N ? 1) such that yk <= y <= yk+1
        int sPointLeft = -1;
        for(int i = 0; i < nRules - 1; ++i){
            //cout << "i = " << i << " : yn = " << yn << ", lowerY(i) = " << lowerY(i) << ", lowerY(i + 1) = " << lowerY(i + 1) << endl;
            if(yn >= lowerY(i) && yn <= lowerY(i + 1)){
                sPointLeft = i;
                //cout << "sPointLeft:\n" << sPointLeft << endl;
                break;
            }
        }
        //cout << "sPointLeft:\n" << sPointLeft << endl;

        // d) Compute
        VectorXd fn(nRules);
        for(int i = 0; i < nRules; ++i)
            if(i <= sPointLeft)
                fn(i) = sortedF(i, 1);
            else
                if(i > sPointLeft)
                    fn(i) = sortedF(i, 0);

        //cout << "fn:\n" << fn << endl;
        //cout << "sum(fn.transpose()):\n" << sum(fn.transpose()) << endl;

        double ynPrime;
        if(sum(fn.transpose())(0) == 0)
            ynPrime = 0;
        else
            ynPrime = sum((lowerY.cwiseProduct(fn)).transpose())(0) / sum(fn.transpose())(0);

        //cout << "ynPrime:\n" << ynPrime << endl;

        // e) if yn==ynPrime stop else go to c)
        //cout << "abs(yn - ynPrime):\n" << abs(yn - ynPrime) << endl;
        if(abs(yn - ynPrime) < pow(10, -3)){
            yLeft = yn;
            break;
        }
        else
            yn = ynPrime;
    }

    /* KM Algorithm for Computing Y Right */

    // a) Sort Y matrix
    VectorXd upperY;
    upperY = Y.col(1);

    //cout << "upperY:\n" << lowerY.transpose() << endl;
    //cout << "F:\n" << F.transpose() << endl;

    ind = sort(upperY, F);

    upperY = ind.col(0);
    sortedF = ind.rightCols(2);

    //cout << "sorted_upperY:\n" << lowerY.transpose() << endl;
    //cout << "sortedF:\n" << sortedF.transpose() << endl;

    // b) Initialize fn by setting and computing
    FOrt = sum(F) / 2;

    //cout << "FOrt:\n" << FOrt << endl;

    isZero = (sum(FOrt)(0) == 0);

    //cout << "isZero:\n" << isZero << endl;

    if(isZero)
        yn = 0;
    else
        yn = sum(upperY.cwiseProduct(FOrt))(0) / sum(FOrt)(0);

    //cout << "yn:\n" << yn << endl;

    while(1){
        // c) Find switch point k (1 <= k <= N ? 1) such that yk <= y <= yk+1
        int sPointRight = -1;
        for(int i = 0; i < nRules - 1; ++i){
            //cout << "i = " << i << " : yn = " << yn << ", lowerY(i) = " << lowerY(i) << ", lowerY(i + 1) = " << lowerY(i + 1) << endl;
            if(yn >= upperY(i) && yn <= upperY(i + 1)){
                sPointRight = i;
                //cout << "sPointRight:\n" << sPointRight << endl;
                break;
            }
        }
        //cout << "sPointRight:\n" << sPointRight << endl;

        // d) Compute
        VectorXd fn(nRules);
        for(int i = 0; i < nRules; ++i)
            if(i <= sPointRight)
                fn(i) = sortedF(i, 0);
            else
                if(i > sPointRight)
                    fn(i) = sortedF(i, 1);

        //cout << "fn:\n" << fn << endl;
        //cout << "sum(fn.transpose()):\n" << sum(fn.transpose()) << endl;

        double ynPrime;
        if(sum(fn.transpose())(0) == 0)
            ynPrime = 0;
        else
            ynPrime = sum((upperY.cwiseProduct(fn)).transpose())(0) / sum(fn.transpose())(0);

        //cout << "ynPrime:\n" << ynPrime << endl;

        // e) if yn==ynPrime stop else go to c)
        //cout << "abs(yn - ynPrime):\n" << abs(yn - ynPrime) << endl;
        if(abs(yn - ynPrime) < pow(10, -3)){
            yRight = yn;
            break;
        }
        else
            yn = ynPrime;
    }

    /* Compute the output */
    //cout << "yLeft:\n" << yLeft << endl;
    //cout << "yRight:\n" << yRight << endl;
    return (yLeft + yRight) / 2;
}

MatrixXd DIT2FLC::sort(VectorXd v, MatrixXd M){
    MatrixXd result = MatrixXd::Zero(v.size(), 3);

    double swap;
    for(int i = 1; i < v.size(); ++i)
      for(int j = 0 ; j < v.size() - 1; ++j)
        if (v(j) > v(j + 1)){
          swap = v(j + 1);
          v(j + 1) = v(j);
          v(j) = swap;

          swap = M(j + 1, 0);
          M(j + 1, 0) = M(j, 0);
          M(j, 0) = swap;
          swap = M(j + 1, 1);
          M(j + 1, 1) = M(j, 1);
          M(j, 1) = swap;
        }

    result.col(0) = v;
    result.col(1) = M.col(0);
    result.col(2) = M.col(1);

    return result;
}

VectorXd DIT2FLC::sum(MatrixXd M){
    VectorXd result = VectorXd::Zero(M.rows());

    for(int i = 0 ; i < M.rows(); ++i)
      for(int j = 0 ; j < M.cols(); ++j)
          result(i) += M(i, j);

    return result;
}

int main(int argc, char** argv){
    cout << "[DIT2FLC] DI-T2-FLC position controller is running..." << endl;

    DIT2FLC* controller = new DIT2FLC(argc, argv);

    controller->run();
}
