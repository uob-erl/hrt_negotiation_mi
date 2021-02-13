#ifndef NEGOTIATION_MI_H
#define NEGOTIATION_MI_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>

class NegotiationMI
{
public:
  NegotiationMI(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~NegotiationMI();

  int negotiationAlgorithm();


private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber current_loa_sub_, loa_utility_delta_sub_, human_suggested_loa_sub , ai_suggested_loa_sub_, goal_directed_motion_error_sub_;
  ros::Publisher hmi_neg_status_pub_, hmi_neg_time_pub_, negotiated_loa_pub_;
  ros::Publisher hmi_loa_ai_pub_, hmi_loa_human_pub_, loa_delta_pub_;
  ros::Timer negotiation_algorithm_timer_;

  void currentLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void loaUtilityDeltaCallback(const std_msgs::Float64::ConstPtr& msg);
  void humanLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void aiLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void timerNegotiationCallback(const ros::TimerEvent&);
  void goalDirectedErrorCallback(const std_msgs::Float64::ConstPtr& msg);

  std_msgs::Bool hmi_neg_status_;
  std_msgs::Int8 negotiated_loa_;
  std_msgs::Int8 hmi_loa_ai_;
  std_msgs::Int8 hmi_loa_human_;
  std_msgs::Float64 hmi_neg_time_, utility_delta_msg_;
    
  /* loa states definition:
		-1 	no input
		0 	stop mode; no LOA; robot just stoped without accepting commands from either agent.
		1 	teleoperation
		2 	autonomy
  */
  int human_suggested_loa_, ai_suggested_loa_, current_loa_;
  int human_suggested_loa_history_, ai_suggested_loa_history_;
  double goal_directed_error_, loa_utility_delta_, concession_rate_, utility_current_loa_ , utility_alternative_loa_, utility_delta_;
  double negotiation_deadline_, time_negotiation_started_;
  bool negotiation_is_active_, negotiation_enabled_;     

};

#endif // NEGOTIATION_MI_H