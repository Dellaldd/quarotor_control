#include "boost/regex.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/VFR_HUD.h>
#include "OffboardWrapper.h"

struct PathPoint{
    double time;
    Eigen::Vector3d position;
    double psi;
};

enum state1{
    CIRCLE, HOVER1
};

class SendCommandCircle{
    private:
        ros::NodeHandle nh;
        const double FREQUENCY = 0.05; //频率
        bool rc_state = 0;
        bool start_state = 1;
        string current_state;
        string traj_path;
        std::vector<std::string> split_vec(std::string str,std::string s);
        vector<PathPoint> csv_data_;
        double currentX, currentY, currentZ;
        double start_time, start_planning_time;
        int data_ptr = 0;
        geometry_msgs::PoseStamped cmd, start_pose;
        


    public:
        SendCommandCircle(string path, geometry_msgs::PoseStamped hover_state);
        ~SendCommandCircle();
        ros::Publisher pos_cmd_pub;
        ros::Subscriber pos_cmd_sub;
        ros::Timer cmd_timer;
        
        void loadtrajectorydata();
        void cmdCallback(const ros::TimerEvent& event);
        void rc_state_Callback_cmd(const mavros_msgs::VFR_HUD::ConstPtr &msg);
        
};