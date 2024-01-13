#include "OffboardWrapper.h"

using namespace std;

OffboardWrapper::OffboardWrapper(geometry_msgs::PoseStamped position_setpoint, std::string id, std::string node_id, std::string dataset)
{
  uav_id = id;
  dataset_address = dataset;
  // Publisher
  m_Publisher.wrapper_local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(uav_id + "/mavros/setpoint_position/local", 10);
  m_Publisher.wrapper_vision_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(uav_id + "/mavros/vision_pose/pose", 10);
  m_Publisher.wrapper_attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(uav_id + "/mavros/setpoint_raw/attitude", 10);

  m_Publisher.position_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>(node_id + "/position_setpoint", 10);
  m_Publisher.velocity_setpoint_pub = nh.advertise<geometry_msgs::TwistStamped>(node_id + "/velocity_setpoint", 10);
  m_Publisher.attitude_setpoint_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/attitude_setpoint", 10);
  m_Publisher.attitude_cureuler_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/attitude_euler", 10);
  m_Publisher.wrapper_status_pub = nh.advertise<std_msgs::Bool>(node_id + "/status", 10);
  m_Publisher.wrapper_new_velocity_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/new_velocity", 10);

  // subscriber();
  subscriber();
  start_position_setpoint_ = position_setpoint;
  current_status_ = HOVER;

  psi_cmd_ = 0;
  hover_flag = 1;
  planning_flag = 1;
  end_flag = 1;
  // cmd_flag = 0;
}

OffboardWrapper::~OffboardWrapper()
{
}

void OffboardWrapper::getEndPoint()
{
  ifstream fin(dataset_address);
  string line, word;
  vector<string> row;
  vector<vector<string>> content;
  int row_num = 0, col_num = 0;

  while (getline(fin, line))
  {
    row.clear();
    stringstream str(line);
    col_num = 0;
    while (getline(str, word, ','))
    {
      row.push_back(word);
      content.push_back(row);
      col_num += 1;
    }
    row_num += 1;
  }
  // std::cout << col_num << std::endl;
  end_position_setpoint_.pose.position.x = stof(content[(row_num)*col_num-2][1]);
  end_position_setpoint_.pose.position.y = stof(content[(row_num)*col_num-2][2]);
  end_position_setpoint_.pose.position.z = stof(content[(row_num)*col_num-2][3]);
  fin.close();
}

void OffboardWrapper::isAtSetpoint()
{
  Eigen::Vector3d hp_(start_position_setpoint_.pose.position.x,
                      start_position_setpoint_.pose.position.y,
                      start_position_setpoint_.pose.position.z);
  Eigen::Vector3d dis_ = wrap_data.wrapper_current_position_ - hp_;
  if (dis_.norm() < 0.1)
  {
    if (hover_flag && wrap_data.rc_state == 1)
    {
      start_hover_t = ros::Time::now();
      hover_flag = 0;
    }
    if(wrap_data.rc_state != 1)
    {
      hover_flag = 1;
      // cmd_flag = 0;
    }
    if ((ros::Time::now() - start_hover_t).toSec() >= 5.0 && wrap_data.rc_state == 1){
        // current_status_ = HOVER; // enter planning
      current_status_ = PLANNING;
      // cmd_flag = 1;
    }
     
  }
  else
    current_status_ = HOVER;
}

void OffboardWrapper::topicPublish()
{
  wrap_data.thrust_attitude_cmd_.header.frame_id = "base_footprint";
  wrap_data.thrust_attitude_cmd_.header.stamp = ros::Time::now();
  m_Publisher.wrapper_new_velocity_pub.publish(wrap_data.pub_new_velocity);
  m_Publisher.wrapper_attitude_pub_.publish(wrap_data.thrust_attitude_cmd_);
  
  m_Publisher.position_setpoint_pub.publish(wrap_data.pub_setpoint_position_);
  // std::cout << wrap_data.pub_setpoint_position_ << std::endl;
  m_Publisher.velocity_setpoint_pub.publish(wrap_data.pub_setpoint_velocity_);
  m_Publisher.attitude_setpoint_pub.publish(wrap_data.pub_setpoint_attitude_);
  m_Publisher.attitude_cureuler_pub.publish(wrap_data.pub_euler_attitude_);

  wrap_data.is_ready_.data = current_status_;
  m_Publisher.wrapper_status_pub.publish(wrap_data.is_ready_);


}

void OffboardWrapper::subscriber()
{
  // m_Subscriber.wrapper_state_sub_ = nh.subscribe<mavros_msgs::State>(uav_id + "/mavros/state",
  //                                                                    10,
  //                                                                    &OffboardWrapper::stateCallback,
  //                                                                    this);
  // in real
  m_Subscriber.wrapper_rc_state_sub_ = nh.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud",
                                                      10, 
                                                      &OffboardWrapper::rc_state_Callback, 
                                                      this);
  m_Subscriber.wrapper_vrpn_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/jiahao1/pose",
                                                      10, 
                                                      &OffboardWrapper::visualCallback, 
                                                      this);
  // m_Subscriber.wrapper_vrpn_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/imu_ekf/body/pose",
  //                                                     10, 
  //                                                     &OffboardWrapper::visualCallback, 
  //                                                     this);
  // in simulator
  // m_Subscriber.wrapper_vrpn_sub_ = nh.subscribe<nav_msgs::Odometry>(uav_id + "/mavros/local_position/odom",
  //                                                                   10,
  //                                                                   &OffboardWrapper::visualCallback,
  //                                                                   this);

  // m_Subscriber.wrapper_current_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(uav_id + "/mavros/local_position/pose",
  //                                                                              10,
  //                                                                              &OffboardWrapper::localCallback,
  //                                                                              this);
  m_Subscriber.wrapper_velocity_sub_ = nh.subscribe<geometry_msgs::TwistStamped>("outer_velocity",
                                                                                 10,
                                                                                 &OffboardWrapper::velocityCallback,
                                                                                 this);
  m_Subscriber.wrapper_acc_sub_ = nh.subscribe<geometry_msgs::TwistStamped>("outer_acc",
                                                                            10,
                                                                            &OffboardWrapper::accCallback,
                                                                            this);
  m_Subscriber.wrapper_poscmd_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/planning/pos_cmd",
                                                                            10,
                                                                            &OffboardWrapper::positioncmdCallback,
                                                                            this);
  // arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_id + "/mavros/cmd/arming");
  // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_id + "/mavros/set_mode");
  // offb_set_mode.request.custom_mode = "OFFBOARD";
  // arm_cmd.request.value = true;

//   if(uav_id == "/uav0")
//     m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb1_node/status",
//                                                                    10,
//                                                                    &OffboardWrapper::statusCallback,
//                                                                    this);
//   else
//     m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb_node/status",
//                                                                    10,
//                                                                    &OffboardWrapper::statusCallback,
//                                                                    this);
}

// void OffboardWrapper::stateCallback(const mavros_msgs::State::ConstPtr &msg)
// {
//   wrapper_current_state_ = *msg;
//   wrap_data.current_state_ = wrapper_current_state_.mode;
// }
void OffboardWrapper::rc_state_Callback(const mavros_msgs::VFR_HUD::ConstPtr &msg){
  wrap_data.rc_state = msg->heading;
  // std::cout<<msg->heading<<std::endl;
}
void OffboardWrapper::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  geometry_msgs::TwistStamped wrapper_current_velo;
  wrapper_current_velo = *msg;
  // x y switch from betaflight!!!!!!!!!!!
  Eigen::Vector3d cur_velocity(wrapper_current_velo.twist.linear.x,
                               wrapper_current_velo.twist.linear.y,
                               wrapper_current_velo.twist.linear.z);
  wrap_data.wrapper_current_velocity_ = cur_velocity;
}

void OffboardWrapper::accCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  geometry_msgs::TwistStamped wrapper_current_acc;
  wrapper_current_acc = *msg;
  // x y switch from betaflight!!!!!!!!!!!
  Eigen::Vector3d cur_acc(wrapper_current_acc.twist.linear.x,
                               wrapper_current_acc.twist.linear.y,
                               wrapper_current_acc.twist.linear.z);
  wrap_data.wrapper_current_acc_ = cur_acc;
}

void OffboardWrapper::positioncmdCallback(const geometry_msgs::PoseStamped::ConstPtr &cmd)
{
  planning_position_setpoint_.pose.position.x = cmd->pose.position.x;
  planning_position_setpoint_.pose.position.y = cmd->pose.position.y;
  planning_position_setpoint_.pose.position.z = cmd->pose.position.z;
  psi_cmd_ = 0;
}

// void OffboardWrapper::localCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//   geometry_msgs::PoseStamped wrapper_current_local;
//   tf::Quaternion rq;
//   wrapper_current_local = *msg;

//   // Vector3d cur_position_(wrapper_current_local.pose.position.x,
//   //                        wrapper_current_local.pose.position.y,
//   //                        wrapper_current_local.pose.position.z);

//   // wrap_data.wrapper_current_position_ = cur_position_;

//   tf::quaternionMsgToTF(wrapper_current_local.pose.orientation, rq);
//   tf::Matrix3x3(rq).getRPY(wrap_data.wrapper_current_attitude_[0],
//                            wrap_data.wrapper_current_attitude_[1],
//                            wrap_data.wrapper_current_attitude_[2]);
// }
// in real
 void OffboardWrapper::visualCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
   geometry_msgs::PoseStamped wrapper_current_vrpn_ = *msg;
   tf::Quaternion rq;
   Vector3d cur_position_(wrapper_current_vrpn_.pose.position.x,
                          wrapper_current_vrpn_.pose.position.y,
                          wrapper_current_vrpn_.pose.position.z);
  // Vector3d cur_velosity_;
  // cur_velosity_ = (cur_position_ - wrap_data.wrapper_last_position_) / (ros::Time::now() - wrap_data.last_v_time).toNSec();
  // wrap_data.last_v_time = ros::Time::now();
  // wrap_data.wrapper_last_position_ = cur_position_;
  // wrap_data.wrapper_current_velocity_ = cur_velosity_;
  // std::cout << (ros::Time::now() - wrap_data.last_v_time).toNSec() << std::endl;

  wrap_data.wrapper_current_position_ = cur_position_;

  tf::quaternionMsgToTF(wrapper_current_vrpn_.pose.orientation, rq);
  tf::Matrix3x3(rq).getRPY(wrap_data.wrapper_current_attitude_[0],
                           wrap_data.wrapper_current_attitude_[1],
                           wrap_data.wrapper_current_attitude_[2]);

  // geometry_msgs::PoseStamped wrapper_vision_pos_ = wrapper_current_vrpn_;
  // wrapper_vision_pos_.header.stamp = ros::Time::now();
  // m_Publisher.wrapper_vision_pos_pub_.publish(wrapper_vision_pos_);
}
// in simulator
// void OffboardWrapper::visualCallback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//   nav_msgs::Odometry wrapper_current_vrpn_ = *msg;
//   Vector3d cur_position_(wrapper_current_vrpn_.pose.pose.position.x,
//                          wrapper_current_vrpn_.pose.pose.position.y,
//                          wrapper_current_vrpn_.pose.pose.position.z);

//   wrap_data.wrapper_current_position_ = cur_position_;
// }

void OffboardWrapper::statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ready_flag = (*msg).data;
}

void OffboardWrapper::run()
{
  wrap_data.begin_time = ros::Time::now();
  QuadrotorFeedbackController c1(start_position_setpoint_, &wrap_data);
  cout << "22222222" << endl;
  // QuadrotorAggressiveController c2(&wrap_data);
  // getEndPoint(); // end_position_setpoint_ from this function
  //c2.readCsvData(dataset_address);
  // QuadrotorFeedbackController c3(end_position_setpoint_, &wrap_data);

  ros::Rate rate(LOOP_FREQUENCY);

  while (ros::ok())
  {
    // std::cout << "end_position_setpoint_.pose.position.x: " << end_position_setpoint_.pose.position.x << std::endl;
    // std::cout << "end_position_setpoint_.pose.position.y: " << end_position_setpoint_.pose.position.y << std::endl;
    // std::cout << "end_position_setpoint_.pose.position.z: " << end_position_setpoint_.pose.position.z << std::endl;
    // start_planning_t_ = ros::Time::now();
    // if (wrap_data.current_state_ != OFFBOARD)
    // {
    //   if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    //     ROS_INFO("OFFBOARD enabled");
    // }
    // else
    // {
    //   if (!wrapper_current_state_.armed)
    //   {
    //     if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    //       ROS_INFO("Vehicle armed");
    //   }
    // }

    switch (current_status_)
    {
    case HOVER:
      printf("enter hover!!\n");
      isAtSetpoint();
      c1.loadLatestData();
      if(!wrap_data.rc_state){
        c1.reset_error_sum_both_pv();
        ROS_INFO("RESET I");
      }
      c1.positionControlFeedback();
      c1.velocityControlFeedback();
      start_planning_t_ = ros::Time::now();
      break;

    case READY:
      // printf("enter ready!!\n");
      // // if(ready_flag) current_status_ = PLANNING;
      // current_status_ = PLANNING;
      // c1.loadLatestData();
      // c1.positionControlFeedback();
      // c1.velocityControlFeedback();
      // start_planning_t_ = ros::Time::now();
      break;


    case PLANNING:
      if(!wrap_data.rc_state){
        current_status_ = HOVER;
        ROS_INFO("RESET PLANNING");
      }else
      {
        printf("enter planning!!\n");
        c1.loadLatestData();
        c1.positionPlanningFeedback(planning_position_setpoint_);
        c1.velocityPlanningFeedback(psi_cmd_);
      }
      // if (planning_flag)
      // {
      //   for (int i = 0; i < 5; i++)
      //     c2.thrust_ave_ += wrap_data.thrust_eval[i] / 5;
      //   planning_flag = 0;
      // }
      // c2.current_time_ = (ros::Time::now() - start_planning_t_).toSec();
      // c2.loadFeedforwardData();
      // c2.loadLatestData();
      // c2.aggressiveControl();
      // if (c2.ff_cmd.is_done)
      //   current_status_ = END;

      break;

    case END:
      // printf("aggressive flight is done!!\n");
      // c3.loadLatestData();
      // c3.positionControlFeedback();
      // c3.velocityControlFeedback();
      break;
    default:
      break;
    }
    topicPublish();
    ros::spinOnce();
    rate.sleep();
  }
}
