#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
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
	loa_utility_delta_sub_ = nh_.subscribe("/loa_utility_delta", 5, &NegotiationMI::loaUtilityDeltaCallback, this);
	human_suggested_loa_sub = nh_.subscribe("/human_suggested_loa", 5, &NegotiationMI::humanLoaCallback, this);
	ai_suggested_loa_sub_ = nh_.subscribe("/ai_suggested_loa", 5, &NegotiationMI::aiLoaCallback, this);

	// Publishers AKA ROS node outputs
	negotiated_loa_pub_ = nh_.advertise<std_msgs::Int8>("/negotiated_loa", 1);

	// Negotiation algorithm main callback function based on a timer.
	negotiation_algorithm_timer_ = nh_.createTimer(ros::Duration(0.2), &NegotiationMI::timerNegotiationCallback, this, false, false);

	// negotiation specifc initializations. These can be initialized via .launch file if needed instead hard coding.
	negotiation_deadline_ = 6; // this influenced by delta
	concession_rate_ = 0.2; // This rate mimics the human based on prior data and it means how "pressure feels to concnet as deadline approaches"

	/* loa states definition
			-1 	no input
			0 	stop mode
			1 	teleoperation
			2 	autonomy
		*/
	current_loa_ = 0; // initializing to "stop" mode
	loa_utility_delta_ = 0;
	human_suggested_loa_ = -1; // initializing to "no input"
	ai_suggested_loa_ = -1;
	human_suggested_loa_history_ = -1;
	ai_suggested_loa_history_ = -1;
	loa_changed_ = false;
	negotiation_enabled_ = false;
	negotiation_is_active_ = false;
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

		// differentiate: human started negotiation
		if (human_suggested_loa_history_ > 0)
		{
			// check if human gave in; equal to changing offer
			if (human_suggested_loa > 0 && human_suggested_loa != human_suggested_loa_history_)
			{
				agreed_loa = human_suggested_loa;
			}
			// if human did not give in, check target utility and time
			else
			{
				// determine duration since negotiation start
				double current_negotiation_duration = ros::Time::now().toSec() - time_negotiation_started_;
				ROS_INFO("current_negotiation_duration: %f",current_negotiation_duration);

				// determine if target utility reached other option's utility
				if (abs(loa_utility_delta) > 0)
				{
					ROS_INFO("Has the delta deminsihed: %f", (1 - pow(current_negotiation_duration / negotiation_deadline_, 1 / concession_rate_)) ) ;
					if ( (1 - pow(current_negotiation_duration / negotiation_deadline_, 1 / concession_rate_)) < (1 - abs(loa_utility_delta)))
					{
						agreed_loa = human_suggested_loa_history_;
					}
				}

				else if (current_negotiation_duration > negotiation_deadline_)
				{
					// if deadline is reached and human offer available -> human choice is applied
					agreed_loa = human_suggested_loa_history_;
				}
			}
		}
		else // automation started negotiation
		{
			// determine duration since negotiation start
			double current_negotiation_duration = ros::Time::now().toSec() - time_negotiation_started_;
			ROS_INFO("current_negotiation_duration: %f",current_negotiation_duration);
			

			// check if deadline is reached
			if (current_negotiation_duration > negotiation_deadline_)
			{
				// if deadline is reached and no human offer found -> automation choice is applied
				agreed_loa = ai_suggested_loa_history_;
			}
			// check for new human offers
			if (human_suggested_loa > 0)
			{
				// check if agreement is found
				if (human_suggested_loa == ai_suggested_loa_history_)
				{
					agreed_loa = human_suggested_loa;
				}
				// if no agreement found update human offer history
				else
				{
					human_suggested_loa_history_ = human_suggested_loa;
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
		}
	}

	// *****case: awaiting start of negotiation (if negotiation is enabled but not active)
	// -> no negotiation is running: check human offer and automation offer
	else if (negotiation_enabled)
	{
		// human is initiating negotiation
		if (human_suggested_loa>0 && (human_suggested_loa != current_loa) )
		{
			negotiation_is_active_ = true;
			//time_init_ = ros::Time::now(); // comenting this out for now in order to compile
			time_negotiation_started_ = ros::Time::now().toSec();
			ROS_INFO("time_negotiation_started_: %f", time_negotiation_started_);
			
			human_suggested_loa_history_ = human_suggested_loa;
		}
		// automation is initiating negotiation
		else if ( ai_suggested_loa>0 && (ai_suggested_loa != current_loa) )
		{
			negotiation_is_active_ = true;
			/// time_init_ = ros::Time::now();  // comenting this out for now in order to compile
			time_negotiation_started_ = ros::Time::now().toSec();
			ROS_INFO("time_negotiation_started_: %f", time_negotiation_started_);
			ai_suggested_loa_history_ = ai_suggested_loa_;
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