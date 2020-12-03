#ifndef NEGOTIATION_MI_H
#define NEGOTIATION_MI_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <math.h>

class NegotiationMI
{
public:
  NegotiationMI(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~NegotiationMI();

  int negotiationAlgorithm();


private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber current_loa_sub_, loa_utility_delta_sub_, human_suggested_loa_sub , ai_suggested_loa_sub_;
  ros::Publisher loa_changed_pub_, negotiated_loa_pub_;
  ros::Timer negotiation_algorithm_timer_;

  void currentLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void loaUtilityDeltaCallback(const std_msgs::Float64::ConstPtr& msg);
  void humanLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void aiLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void timerNegotiationCallback(const ros::TimerEvent&);

  std_msgs::Int8 previous_loa_, negotiated_loa_;
  
  /* loa states definition:
		-1 	no input
		0 	stop mode; no LOA; robot just stoped without accepting commands from either agent.
		1 	teleoperation
		2 	autonomy
  */
  int human_suggested_loa_, ai_suggested_loa_, current_loa_;
  int human_suggested_loa_history_, ai_suggested_loa_history_;
  double loa_utility_delta_, negotiation_deadline_;
  bool negotiation_is_active_, negotiation_enabled_, concession_rate_, loa_changed_; 
    

};

#endif // NEGOTIATION_MI_H