#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <math.h>

#include "negotiation_mi.h"

using namespace std;

// costructor for initializing values. This will be important at some point.
NegotiationMI::NegotiationMI(ros::NodeHandle nh, ros::NodeHandle private_nh)
{

	nh_ = nh;
	pnh_ = private_nh;

	// ros::NodeHandle private_nh("negotiation_mi");
	// Just n example how we read initializations from launch files
	// private_nh.param("noise_period", noise_period_, 30.0);

	// Subscribers AKA ROS node inputs
	current_loa_sub_ = nh_.subscribe("/loa", 5, &NegotiationMI::currentLoaCallback, this);

	// loa_utility_delta msg has to change/be sent whenever situation changes
	// -> msg therefore also important in the negotiation interface to visualize enabled negotiation
	goal_directed_motion_error_sub_ = nh_.subscribe("/goal_directed_motion/error_average", 5, &NegotiationMI::goalDirectedErrorCallback, this);
	loa_utility_delta_sub_ = nh_.subscribe("/loa_utility_delta", 5, &NegotiationMI::loaUtilityDeltaCallback, this);
	human_suggested_loa_sub = nh_.subscribe("/human_suggested_loa", 5, &NegotiationMI::humanLoaCallback, this);
	ai_suggested_loa_sub_ = nh_.subscribe("/ai_suggested_loa", 5, &NegotiationMI::aiLoaCallback, this);

	// Publishers AKA ROS node outputs
	hmi_neg_status_pub_ = nh_.advertise<std_msgs::Bool>("/nemi/negotiation_enabled", 1);
	negotiated_loa_pub_ = nh_.advertise<std_msgs::Int8>("/nemi/negotiated_loa", 1);
	hmi_loa_ai_pub_ = nh_.advertise<std_msgs::Int8>("/nemi/ai_suggested_loa", 1);
	hmi_loa_human_pub_ = nh_.advertise<std_msgs::Int8>("/nemi/human_suggested_loa", 1);
	hmi_neg_time_pub_ = nh_.advertise<std_msgs::Float64>("/nemi/negotiation_time", 1);
	loa_delta_pub_ = nh_.advertise<std_msgs::Float64>("/loa_utility_delta", 1);

	// Negotiation algorithm main callback function based on a timer.
	negotiation_algorithm_timer_ = nh_.createTimer(ros::Duration(0.2), &NegotiationMI::timerNegotiationCallback, this, false, false);

	// negotiation specifc initializations. These can be initialized via .launch file if needed instead hard coding.
	negotiation_deadline_ = 6; //default 6 this influenced by delta
	concession_rate_ = 0.2;	   // default 0.2 This rate mimics the human based on prior data and it means how "pressure feels to concnet as deadline approaches"

	/* loa states definition
			-1 	no input
			0 	stop mode
			1 	teleoperation
			2 	autonomy
		*/
	current_loa_ = 0; // initializing to "stop" mode
	loa_utility_delta_ = 0;
	utility_alternative_loa_ = 0.8;
	human_suggested_loa_ = -1; // initializing to "no input"
	ai_suggested_loa_ = -1;
	human_suggested_loa_history_ = -1;
	ai_suggested_loa_history_ = -1;
	negotiation_enabled_ = false;
	negotiation_is_active_ = false;
	hmi_neg_status_.data = negotiation_enabled_;
	hmi_neg_status_pub_.publish(hmi_neg_status_);
	hmi_neg_time_.data = -1;
	hmi_neg_time_pub_.publish(hmi_neg_time_);
	negotiation_algorithm_timer_.start();
}

NegotiationMI::~NegotiationMI()
{
}

// This reads the up-to-date current LOA, AKA the active LOA and stores it.
// if the new active LOA is different than the previously active one then the LOA has changed
void NegotiationMI::currentLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
	current_loa_ = msg->data;
	ROS_INFO("current_loa: %d", current_loa_);
}

// This reads and stores the up-to-date loa utility delta and enables negotiation
void NegotiationMI::loaUtilityDeltaCallback(const std_msgs::Float64::ConstPtr &msg)
{
	/* 
	 loa_utility_delta msg indicates that new situation/circumstances happened (e.g. robot in a different area under different conditios). 
	 In this new situation each LOA has a utility attached, reinitialized after a change of the situation
	 Hence negotiation must be enabled
	 after a negotiation has finished, negotiation is disabled and can only be reinitialized after a change of the situation
	*/

	loa_utility_delta_ = msg->data;
	ROS_INFO("loa_utility_delta: %f", loa_utility_delta_);
	negotiation_enabled_ = true;
}

// This reads and stores the up-to-date goal directed motion error
void NegotiationMI::goalDirectedErrorCallback(const std_msgs::Float64::ConstPtr &msg)
{
	goal_directed_error_ = msg->data;
	// ROS_INFO("testing: %f",goal_directed_error_);
	utility_current_loa_ = 1 - (goal_directed_error_ * 10);
	utility_delta_ = abs(utility_current_loa_ - utility_alternative_loa_);
	utility_delta_msg_.data = utility_delta_;
	loa_delta_pub_.publish(utility_delta_msg_);
}

// This reads and stores the up-to-date LOA suggested by human/operator
void NegotiationMI::humanLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
	human_suggested_loa_ = msg->data;
	ROS_INFO("human_suggested_loa: %d", human_suggested_loa_);
}

// This reads and stores the up-to-date LOA suggested by the AI
void NegotiationMI::aiLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
	ai_suggested_loa_ = msg->data;
	ROS_INFO("ai_suggested_loa: %d", ai_suggested_loa_);
}

// The main negotiation algorithm were magic happens
// Negotiation algorithm main callback function based on a timer.
void NegotiationMI::timerNegotiationCallback(const ros::TimerEvent &)
{

	// initiating local variables just to be on the safe side
	int human_suggested_loa = human_suggested_loa_;
	int ai_suggested_loa = ai_suggested_loa_;
	int current_loa = current_loa_;
	double loa_utility_delta = loa_utility_delta_;
	bool negotiation_enabled = negotiation_enabled_;

	// ******case: active negotiation (if negotiation is enabled and active)
	// -> negotiation round started: check deadline, opponentBehavior and target utility
	if (negotiation_is_active_)
	{
		// agreed_loa is output of negotiation Algorithm: no agreement -> 0, agreement -> option/LOA
		int agreed_loa = 0;
		// determine duration since negotiation start
		double current_negotiation_duration = ros::Time::now().toSec() - time_negotiation_started_;
		ROS_INFO("current_negotiation_duration: %f", current_negotiation_duration);
		// publish normalized negotiation time
		hmi_neg_time_.data = current_negotiation_duration / negotiation_deadline_;
		hmi_neg_time_pub_.publish(hmi_neg_time_);
		// check if deadline is reached
		if (current_negotiation_duration > negotiation_deadline_)
		{
			ROS_INFO("deadline is reached at %f s", negotiation_deadline_);
			if (human_suggested_loa_history_ > 0)
				agreed_loa = human_suggested_loa_history_;
			else if (ai_suggested_loa_history_ > 0)
				agreed_loa = ai_suggested_loa_history_;
			else
				agreed_loa = current_loa;
		}
		else
		{
			// differentiate: human started negotiation
			if (human_suggested_loa_history_ > 0)
			{
				// check if human gave in (= changing offer) or ai gave in
				if (human_suggested_loa > 0 &&
					(human_suggested_loa != human_suggested_loa_history_ ||
					 human_suggested_loa == ai_suggested_loa))
					agreed_loa = human_suggested_loa;
				// if human did not give in, check target utility and time
				else
				{
					// determine if target utility reached other option's utility
					ROS_INFO("Has the delta diminished: %f", (1 - pow(current_negotiation_duration / negotiation_deadline_, 1 / concession_rate_)));
					if ((1 - pow(current_negotiation_duration / negotiation_deadline_, 1 / concession_rate_)) < (1 - abs(loa_utility_delta)))
						agreed_loa = human_suggested_loa_history_;
				}
			}
			// automation started negotiation: check for new human offers
			else if (human_suggested_loa > 0)
			{
				// check if agreement is found
				if (human_suggested_loa == ai_suggested_loa_history_)
					agreed_loa = human_suggested_loa;
				// if no agreement found update human offer history
				else
				{
					human_suggested_loa_history_ = human_suggested_loa;
					hmi_loa_human_.data = human_suggested_loa;
					hmi_loa_human_pub_.publish(hmi_loa_human_);
				}
			}
		}

		// if agreement found or deadline reached terminate negotiation
		if (agreed_loa > 0)
		{
			// publish agreed loa
			negotiated_loa_.data = agreed_loa;
			negotiated_loa_pub_.publish(negotiated_loa_);
			ROS_INFO("negotiated_loa: %d", negotiated_loa_.data);

			// reset variables
			negotiation_is_active_ = false;
			negotiation_enabled_ = false;
			loa_utility_delta_ = 0;
			human_suggested_loa_ = -1;
			ai_suggested_loa_ = -1;
			human_suggested_loa_history_ = -1;
			ai_suggested_loa_history_ = -1;

			// publish negotiation disabled
			hmi_neg_status_.data = negotiation_enabled_;
			hmi_neg_status_pub_.publish(hmi_neg_status_);
			ROS_INFO("negotiation_enabled: %d", hmi_neg_status_.data);
			// publish negative negotiation time to disable deadline bar graph
			hmi_neg_time_.data = -1;
			hmi_neg_time_pub_.publish(hmi_neg_time_);
		}
	}

	// *****case: awaiting start of negotiation (if negotiation is enabled but not active)
	// -> no negotiation is running: check human offer and automation offer
	else if (negotiation_enabled)
	{
		// publish negotiation enabled once
		if (~hmi_neg_status_.data)
		{
			hmi_neg_status_.data = negotiation_enabled;
			hmi_neg_status_pub_.publish(hmi_neg_status_);
			ROS_INFO("negotiation_enabled: %d", hmi_neg_status_.data);
		}
		/// negotiation is only initiated if loa suggestion different from current loa -> problem?
		// human is initiating negotiation
		if (human_suggested_loa > 0 && (human_suggested_loa != current_loa))
		{
			negotiation_is_active_ = true;
			time_negotiation_started_ = ros::Time::now().toSec();
			ROS_INFO("time_negotiation_started_: %f", time_negotiation_started_);
			human_suggested_loa_history_ = human_suggested_loa;
			hmi_loa_human_.data = human_suggested_loa;
			hmi_loa_human_pub_.publish(hmi_loa_human_);
		}
		// automation is initiating negotiation
		else if (ai_suggested_loa > 0 && (ai_suggested_loa != current_loa))
		{
			negotiation_is_active_ = true;
			time_negotiation_started_ = ros::Time::now().toSec();
			ROS_INFO("time_negotiation_started_: %f", time_negotiation_started_);
			ai_suggested_loa_history_ = ai_suggested_loa_;
			hmi_loa_ai_.data = ai_suggested_loa;
			hmi_loa_ai_pub_.publish(hmi_loa_ai_);
		}
	}

	// ****case: awaiting enabling of negotiation (if negotiation not enabled and not active)
	// -> reset all values except of loa_utility_delta_
	else if (~negotiation_enabled)
	{
		// reset variables
		negotiation_is_active_ = false;
		human_suggested_loa_ = -1;
		ai_suggested_loa_ = -1;
		human_suggested_loa_history_ = -1;
		ai_suggested_loa_history_ = -1;
	}
}