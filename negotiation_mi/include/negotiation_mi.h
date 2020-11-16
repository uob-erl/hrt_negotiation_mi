#ifndef NEGOTIATION_MI_H
#define NEGOTIATION_MI_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>

class NegotiationMI
{
public:
  NegotiationMI(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~NegotiationMI();

  int negotiationAlgorithm();


private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber current_loa_sub_, human_loa_sub_ , ai_loa_sub_;
  ros::Publisher loa_changed_pub_, negotiated_loa_pub_;
  ros::Timer negotiation_algorithm_timer_;

  void currentLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void humanLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void aiLoaCallback(const std_msgs::Int8::ConstPtr& msg);
  void timerNegotiationCallback(const ros::TimerEvent&);

  std_msgs::Int8 current_loa_, previous_loa_, negotiated_loa_;


  int human_suggested_loa_, ai_suggested_loa_;
  bool loa_changed_, negotiation_is_active_; 
};

#endif // NEGOTIATION_MI_H