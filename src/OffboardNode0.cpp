#include "OffboardWrapper.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "offb_node");
  // ros::NodeHandle nh;
  geometry_msgs::PoseStamped g_position_setpoint0, g_position_setpoint1;
  g_position_setpoint0.pose.position.x = -1.5;
  g_position_setpoint0.pose.position.y = 0.0;
  g_position_setpoint0.pose.position.z = 0.6;

  // g_position_setpoint1.pose.position.x = 0;
  // g_position_setpoint1.pose.position.y = 0;
  // g_position_setpoint1.pose.position.z = 0.7;

  // OffboardWrapper wrapper(g_position_setpoint0, "/uav0", "/offb_node", "/home/zhoujin/time_optimal_trajectory/example/result.csv");
  OffboardWrapper wrapper(g_position_setpoint0, "", "/offb_node", "/home/coolpi/quarotor_controll/src/quarotor_feedback_controller/library/quadM_fasterdt15.txt");
  wrapper.run();
  // OffboardWrapper wrapper(g_position_setpoint1, "/uav1");
  // wrapper.run();

  return 0;
}

