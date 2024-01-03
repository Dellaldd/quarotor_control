#include <Eigen/Geometry>
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
// #include <ros/console.h>

ros::Publisher pos_cmd_pub;
ros::Subscriber pos_cmd_sub;

geometry_msgs::PoseStamped cmd;

struct Point {
    float x;
    float y;
    float z;

    Point(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

enum state1{
    START1,HOVER1,HOVER2,PLAN1,END1,STOP1,FREE1
};

const double AMPLITUDE = 1.5; //幅值
const double FREQUENCY = 0.05; //频率
 
Point start(0.0f, 0.6f, 1.0f);
// Point hover(-1.5f, 0.6f, 1.0f);
Point hover(0.0f, 0.6f, 1.0f);
Point plan(0.0f, 0.6f, 1.0f);
Point end1(0.0f, 0.6f, 1.0f);
Point stop(0.0f, 0.6f, 0.05f);

Point SetPoint(0.0f, 0.5f, 1.0f);

bool rc_state;
bool start_state;
static int current_state;

const int numSteps = 500;  //插值步数

Point linearInterpolation(const Point& start, const Point& end, double t){
    Point result(0.0f, 0.0f, 0.0f);
    result.x = start.x + t * (end.x - start.x);
    result.y = start.y + t * (end.y - start.y);
    result.z = start.z + t * (end.z - start.z);

    return result;
}

void cmdCallback(const ros::TimerEvent& event){
    ros::Time time_now = ros::Time::now();
    static ros::Time last_time;

    static float currentX = start.x;
    static float currentY = start.y;
    static float currentZ = start.z;

    static int i = 0;

    if(rc_state)
    {
        if(start_state)
        {
            last_time = ros::Time::now();
            start_state = 0;
        }
        if((ros::Time::now() - last_time).toSec() <= 10.0)
        {
            current_state = START1;
        }else if((ros::Time::now() - last_time).toSec() <= 20.0)
        {
            current_state = HOVER1;
        }else if((ros::Time::now() - last_time).toSec() <= 30.0)
        {
            current_state = HOVER2;
        }else if((ros::Time::now() - last_time).toSec() <= 230.0)
        {
            current_state = PLAN1;
        }else if((ros::Time::now() - last_time).toSec() <= 240.0)
        {
            current_state = END1;
        }else
        {
            current_state = STOP1;
        }
    }else
    {
        start_state = 1;
    }

    if(!start_state && rc_state == 1)
    {
        double dt = (ros::Time::now() - last_time).toSec();
        static double angle = 0.0;
        double step = 0;

        switch(current_state)
        {
            case START1:
                currentX = start.x;
                currentY = start.y;
                currentZ = start.z;
                i = 0;
                break;
            case HOVER1:
                i++;
                step = static_cast<double>(i) / numSteps;
                SetPoint = linearInterpolation(start, hover, step);
                currentX = SetPoint.x;
                currentY = SetPoint.y;
                currentZ = SetPoint.z;
                break;
            case HOVER2:
                static int j = 0;
                j++;
                if(j >= numSteps)
                {
                    j = numSteps;
                }
                step = static_cast<double>(j) / numSteps;
                SetPoint = linearInterpolation(hover, start, step);
                currentX = SetPoint.x;
                currentY = SetPoint.y;
                currentZ = SetPoint.z;
                break;
            case PLAN1:
                angle = 2.0 * M_PI * FREQUENCY * dt;
                // currentX = AMPLITUDE * sin(angle); //angle + 初始位置
                currentX = plan.x;
                currentY = plan.y;
                currentZ = plan.z;
                break;
            case END1:
                currentX = end1.x;
                currentY = end1.y;
                currentZ = end1.z;
                i = 0;
                break;
            case STOP1:
                i++;
                step = static_cast<double>(i) / numSteps;
                SetPoint = linearInterpolation(end1, stop, step);
                currentX = SetPoint.x;
                currentY = SetPoint.y;
                currentZ = SetPoint.z;
                break;
            default:
                break;
        }
    }
    if(i >= numSteps)
    {
        i = numSteps;
    }

    cmd.pose.position.x = currentX;
    cmd.pose.position.y = currentY;
    cmd.pose.position.z = currentZ;
    pos_cmd_pub.publish(cmd);
    ROS_INFO("x: [%f]", cmd.pose.position.x);
    ROS_INFO("y: [%f]", cmd.pose.position.y);
    ROS_INFO("z: [%f]", cmd.pose.position.z);
}

// void cmdCallback(const ros::TimerEvent& event) {
    
//     std::vector<Point> points;

//     static float dx = 0;
//     static float dy = 0;
//     static float dz = 0;


//     static float currentX = start.x;
//     static float currentY = start.y;
//     static float currentZ = start.z;

//     float xIncrement = 0;
//     float yIncrement = 0;
//     float zIncrement = 0;


//     ros::Time time_now = ros::Time::now();
//     static ros::Time last_time;
//     if(rc_state)
//     {
//         switch(current_state)
//         {
//             case START1:
//                 currentX = start.x;
//                 currentY = start.y;
//                 currentZ = start.z;
//                 start_flag = 1;
//                 // sleep(5);
//                 current_state = END1;
//                 break;
//             case HOVER1:
//                 dx = hover.x - start.x;
//                 dy = hover.y - start.y;
//                 dz = hover.z - start.z;
//                 currentX = start.x;
//                 currentY = start.y;
//                 currentZ = start.z;
//                 current_state = END1;
//                 break;
//             case STOP1:
//                 dx = stop.x - hover.x;
//                 dy = stop.y - hover.y;
//                 dz = stop.z - hover.z;
//                 currentX = hover.x;
//                 currentY = hover.y;
//                 currentZ = hover.z;
//                 current_state = END1;
//                 break;
//             case END1:
//                 xIncrement = dx / stepSize;
//                 yIncrement = dy / stepSize;
//                 zIncrement = dz / stepSize;

//                 currentX += xIncrement;
//                 currentY += yIncrement;
//                 currentZ += zIncrement;
//                 break;
//             case INIT1:
//                 dx = 0;
//                 dy = 0;
//                 dz = 0;
//                 break;
//             default:
//                 break;
//         }


//         if(currentX >= 1.5 && stop_flag == 0)
//         {
//             stop_flag = 1;
//             timer_stop = 1;
//             current_state = INIT1;
//         }

//         if(currentZ < 0.05)
//         {
//             current_state = INIT1;
//         }

//         if(start_flag == 1 && stop_flag == 0)
//         {
//             if(start_state)
//             {
//                 last_time = ros::Time::now();
//                 start_state = 0;
//             }
//             if((ros::Time::now() - last_time).toSec() >= 15.0)
//             {
//                 start_flag = 0;
//                 // timer_stop = 1;
//                 current_state = HOVER1;
//             }
//         }

//         if(timer_stop == 1 && stop_flag == 1)
//         {
//             if(stop_state)
//             {
//                 last_time = ros::Time::now();
//                 stop_state = 0;
//             }
//             if((ros::Time::now() - last_time).toSec() >= 7.0)
//             {
//                 timer_stop = 0;
//                 // stop_flag = 0;
//                 current_state = STOP1;
//             }
//         }

//         cmd.pose.position.x = currentX;
//         cmd.pose.position.y = currentY;
//         cmd.pose.position.z = currentZ;
//         pos_cmd_pub.publish(cmd);
//         ROS_INFO("x: [%f]", cmd.pose.position.x);
//         ROS_INFO("y: [%f]", cmd.pose.position.y);
//         ROS_INFO("z: [%f]", cmd.pose.position.z);

//     }else{
//         currentX = stop.x;
//         currentY = stop.y;
//         currentZ = stop.z;
//         cmd.pose.position.x = currentX;
//         cmd.pose.position.y = currentY;
//         cmd.pose.position.z = currentZ;
//         pos_cmd_pub.publish(cmd);
//         ROS_INFO("stop x: [%f]", cmd.pose.position.x);
//         ROS_INFO("stop y: [%f]", cmd.pose.position.y);
//         ROS_INFO("stop z: [%f]", cmd.pose.position.z);
//     }

    

//     // ROS_INFO("yaw: [%f]",  cmd.yaw );
// }

void rc_state_Callback_cmd(const mavros_msgs::VFR_HUD::ConstPtr &msg)
{
    rc_state = msg->heading;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cmd_test");
    ros::NodeHandle nh;
    rc_state = 0;
    start_state = 1;
    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
    
    pos_cmd_sub = nh.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud",10,&rc_state_Callback_cmd);
    pos_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/planning/pos_cmd", 50);
    
    ros::spin();

}



