
#include "SendCommandCircle.h"

using namespace std;

SendCommandCircle::SendCommandCircle(string path, geometry_msgs::PoseStamped hover_state){
    current_state = HOVER;
    traj_path = path;

    start_pose = hover_state;
    loadtrajectorydata();

    cmd_timer = nh.createTimer(ros::Duration(0.01), &SendCommandCircle::cmdCallback, this);
    pos_cmd_sub = nh.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud",10,&SendCommandCircle::rc_state_Callback_cmd, this);
    pos_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/planning/pos_cmd", 50);
    
}

SendCommandCircle::~SendCommandCircle()
{
}

void SendCommandCircle::cmdCallback(const ros::TimerEvent& event){
    double current_time = ros::Time::now().toSec();

    if(rc_state){
        if(start_state){
            start_time = current_time;
            start_state = 0;
        }
    }else
        start_state = 1;


    // HOVER STATE
    if(!start_state && rc_state == 1 && current_state == HOVER){

        if(current_time - start_time > 10){
            cmd = start_pose;
            start_planning_time = current_time;
            current_state == CIRCLE;
        }
    }

    // CIRCLE STATE
    if(!start_state && rc_state == 1 && current_state == CIRCLE){
        double dt = current_time - start_planning_time;
        if(dt > csv_data_[data_ptr].time){
            cmd.pose.position.x = csv_data_[data_ptr].position(0);
            cmd.pose.position.y = csv_data_[data_ptr].position(1);
            cmd.pose.position.z = csv_data_[data_ptr].position(2);
            tf::Quaternion oq_;
            oq_.setRPY(0, 0, csv_data_[data_ptr].psi);
            cmd.pose.orientation.x = oq_.x();
            cmd.pose.orientation.y = oq_.y();
            cmd.pose.orientation.z = oq_.z();
            cmd.pose.orientation.w = oq_.w();
            data_ptr ++;
        }
        if(data_ptr == csv_data_.size())
            data_ptr --;
    }
    
    // pulish
    cmd.header.stamp = ros::Time::now();
    pos_cmd_pub.publish(cmd);
}


void SendCommandCircle::rc_state_Callback_cmd(const mavros_msgs::VFR_HUD::ConstPtr &msg)
{
    rc_state = msg->heading;
}

std::vector<std::string> SendCommandCircle::split_vec(std::string str,std::string s)
{
    boost::regex reg(s.c_str());
    
    std::vector<std::string> vec;
    boost::sregex_token_iterator it(str.begin(),str.end(),reg,-1);
    boost::sregex_token_iterator end;
    while(it!=end)
    {
        vec.push_back(*it++);
    }
    return vec;
}

void SendCommandCircle::loadtrajectorydata(){

  ifstream ifs_traj;

  ifs_traj.open(traj_path, ios::in);  //读取文件
  if(!ifs_traj.is_open()){
    std::cout << "gt ifstream open file error!\n";
  }

  string line_gt;
  string s_gt = " ";
  vector<string> lines_gt;
  while(getline(ifs_traj, line_gt))    
    lines_gt.push_back(line_gt);   
  ifs_traj.close();

  PathPoint pathpoint;
  vector<std::string> vec_gt;

  for(int i=1; i<lines_gt.size(); ++i){
    vec_gt = split_vec(lines_gt[i],s_gt);
    pathpoint.time = std::stod(vec_gt[0]);
    pathpoint.position = Eigen::Vector3d(std::stod(vec_gt[1]), std::stod(vec_gt[2]), std::stod(vec_gt[3]));
    pathpoint.psi = std::stod(vec_gt[4]);
    csv_data_.push_back(pathpoint);
  }

  ROS_INFO("--------------------FINISH READ CSV DATA--------------------");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_test");
    string trajpath = "/home/ldd/quarotor_controller/src/quarotor_control/library/circle_yaw.txt";
    geometry_msgs::PoseStamped hover_state;

    hover_state.pose.position.x = 0;
    hover_state.pose.position.y = 0;
    hover_state.pose.position.z = 1;

    SendCommandCircle sendcommandcircle(trajpath, hover_state); 
    ros::spin();
}
