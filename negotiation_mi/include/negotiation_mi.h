#ifndef NEGOTIATION_MI_H
#define NEGOTIATION_MI_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <time.h>
#include <duration.h>
#include <math.h>

class NegotiationMI
{
public:
  NegotiationMI(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~NegotiationMI();

  int negotiationAlgorithm();


private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber current_loa_sub_, loa_utility_delta_sub_, human_loa_sub_ , ai_loa_sub_;
  ros::Publisher loa_changed_pub_, negotiated_loa_pub_;
  ros::Timer negotiation_algorithm_timer_;

  void currentLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void loaUtilityDeltaCallback(const std_msgs::Float64::ConstPtr& msg);
  void humanLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void aiLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void timerNegotiationCallback(const ros::TimerEvent&);

  std_msgs::Int8 current_loa_, previous_loa_, negotiated_loa_;
  std_msgs::Float64 loa_utility_delta_;
  
  /* loa states definition:
		-1 	no input
		0 	stop level autonomy
		1 	teleoperation
		2 	autonomy
  */
  int human_suggested_loa_, ai_suggested_loa_, current_loa_;
  int human_suggested_loa_history_, ai_suggested_loa_history_;
  double loa_utility_delta_;
  bool negotiation_is_active_, negotiation_enabled_; 
  
  ros::Time time_init_;
  
  static const double negotiation_deadline_ = 3;
  static const double concession_rate_ = 0.2;
};

#endif // NEGOTIATION_MI_H