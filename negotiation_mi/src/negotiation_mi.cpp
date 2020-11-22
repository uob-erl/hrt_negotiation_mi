#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <time.h>
#include <duration.h>
#include <math.h>

#include "negotiation_mi.h"

using namespace std;

// costructor for initializing values. This will be important at some point.
NegotiationMI::NegotiationMI(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
        // ros::NodeHandle private_nh("negotiation_mi");
        // Initialise the noise period...
        // private_nh.param("noise_period", noise_period_, 30.0);
        nh_ = nh;
        pnh_ = private_nh;

        // Subscribers AKA ROS node inputs
        current_loa_sub_ = nh_.subscribe("/loa", 5, &NegotiationMI::currentLoaCallback, this);
        loa_utility_delta_sub_ = nh_.subscribe("/loa_utility_delta", 5, &NegotiationMI::loaUtilityDeltaCallback, this);
        human_loa_sub_ = nh_.subscribe("/human_suggested_loa", 5, &NegotiationMI::humanLoaCallback, this);
        ai_loa_sub_ = nh_.subscribe("/ai_suggested_loa", 5, &NegotiationMI::humanLoaCallback, this);
		neg_enabled_sub_ = nh_.subscribe("/negotation_enabled", 5, &NegotiationMI::negEnabledCallback, this);
/* --> Q_simon: is it okay to receive 
			- loa_utility_delta_sub_ (scenario specific estimation)
			- neg_enabled_sub_ (scenario sensitive flag -> only one negoation in each situation;
				alternative: idle time between rounds of negotiation, realisation within NegotiationMI)
			from somewhere else?
			*/

        // Publishers AKA ROS node outputs
        negotiated_loa_pub_ = nh_.advertise<std_msgs::Int8>("/negotiated_loa", 1);

        // Negotiation algorithm main callback function based on a timer.
        negotiation_algorithm_timer_ = nh_.createTimer(ros::Duration(0.2), &NegotiationMI::timerNegotiationCallback, this, false, false);

        current_loa_.data = 0; // initializing to "stop" mode
        loa_utility_delta_ = 0;
		human_suggested_loa_ = 0;
        ai_suggested_loa_ = 0;
		human_suggested_loa_history_ = 0;
        ai_suggested_loa_history_ = 0;
        loa_changed_ = false;
		negotiation_enabled_ = false;
		negotiation_is_active_ = false;
        negotiation_algorithm_timer_.start();
		
		time_init_(0);
}

NegotiationMI::~NegotiationMI()
{
}


/* --> Q_simon: loa_changed_ currently not required, right? */

// This reads the up-to-date current LOA, AKA the active LOA and stores it.
// if the new active LOA is different than the previously active one then the LOA has changed
void NegotiationMI::currentLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
        current_loa_.data = msg->data;

        if (current_loa_.data != previous_loa_.data)
        {
                previous_loa_.data = current_loa_.data;
                loa_changed_ = true; // stores if there is a change e.g. auto to teleop and vice versa
                // loa_changed_pub_.publish(loa_changed_msg_); // if needs publishing we can put it here
        }
}

// This reads and stores the up-to-date loa utility delta
void NegotiationMI::loaUtilityDeltaCallback(const std_msgs::Float64::ConstPtr &msg)
{
        loa_utility_delta_ = (double)msg->data;
}

/* --> Q_simon: possible states of human_suggested_loa are which?
			e.g. 0 for no input, 1 or 2 indicating loa?
			event-based? -> currently assumed
*/
// This reads and stores the up-to-date LOA suggested by human/operator
void NegotiationMI::humanLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
        human_suggested_loa_ = msg->data;
}

// This reads and stores the up-to-date LOA suggested by the AI
void NegotiationMI::aiLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
        ai_suggested_loa_ = msg->data;
}

// This reads and stores the up-to-date negotation enbaled flag
void NegotiationMI::negEnabledCallback(const std_msgs::Bool::ConstPtr &msg)
{
        negotiation_enabled_ = msg->data;
}

// The main negotiation algorithm were magic happens
// Negotiation algorithm main callback function based on a timer.
void NegotiationMI::timerNegotiationCallback(const ros::TimerEvent&)
{

	// initiating local variables just to be on the safe side
	int human_suggested_loa = human_suggested_loa_;
	int ai_suggested_loa = ai_suggested_loa_ ;
	int current_loa = current_loa_.data ;
	double loa_utility_delta = loa_utility_delta_;
	bool negotiation_enabled = negotiation_enabled_;
	
	
	// as soon as negotiation round started check deadline, opponentBehavior and target utility
	if (negotiation_is_active_)
	{
		// agreed_loa is output of negtiationAlgorithm: no agreement -> 0, agreement -> option/LOA
		int agreed_loa = 0;
		// differentiate: human started negotiation
		if (human_suggested_loa_history_)
		{
			// check if human gave in; equal to changing offer
			if (human_suggested_loa && human_suggested_loa != human_suggested_loa_history_)
			{
				agreed_loa = human_suggested_loa;
			}
			// if human did not give in, check target utility and time
			else
			{
				// determine duration since negotiation start
				ros::Duration current_neg_duration = ros::Time::now() - time_init_;
				current_neg_duration_d = current_neg_duration.toSec();
				// determine if target utility reached other option's utility
				if (loa_utility_delta > 0)
				{
					if ((double)(1 - pow(current_neg_duration_d/negotiation_deadline_,1/concession_rate_)) < (double)(1 - loa_utility_delta))
					{
						agreed_loa = human_suggested_loa_history_;
					}
				}
				else if (current_neg_duration_d > negotiation_deadline_)
				{
					// if deadline is reached and human offer available -> human choice is applied
					agreed_loa = human_suggested_loa_history_;
				}				
			}
		}
		else	// automation started negotiation
		{
			// determine duration since negotiation start
			ros::Duration current_neg_duration = ros::Time::now() - time_init_;
			// check if deadline is reached
			if (current_neg_duration.toSec() > negotiation_deadline_)
			{
				// if deadline is reached and no human offer found -> automation choice is applied
				agreed_loa = ai_suggested_loa_history_;
			}
			// check for new human offers
			if (human_suggested_loa > 0 )
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
		if (agreed_loa)
		{
/* --> Q_simon: is publishing syntax correct? not sure about that*/
			// publish agreed loa
			negotiated_loa_.data = agreed_loa;
			negotiated_loa_pub_.publish(negotiated_loa_);
			// reset variables
			negotiation_is_active_ = false;
			loa_utility_delta_ = 0;
			human_suggested_loa_ = 0;
			ai_suggested_loa_ = 0;
			human_suggested_loa_history_ = 0;
			ai_suggested_loa_history_ = 0;
		}
		
	}
	// if no negotiation is running check human offer and automation offer
	else if (negotiation_enabled)
	{
		// human is initiating negotiation
		if (human_suggested_loa && human_suggested_loa != current_loa )
		{
			negotiation_is_active_ = true;
			time_init_ = ros::Time::now();
			human_suggested_loa_history_ = human_suggested_loa;			
		}
		// automation is initiating negotiation
		else if (ai_suggested_loa && ai_suggested_loa != current_loa )
		{
			negotiation_is_active_ = true;
			time_init_ = ros::Time::now();
			ai_suggested_loa_history_ = ai_suggested_loa_;
		}
	}
	// negotiation is not enabled
	else if (~negotiation_enabled)
	{
		// reset variables
		negotiation_is_active_ = false;
		loa_utility_delta_ = 0;
		human_suggested_loa_ = 0;
		ai_suggested_loa_ = 0;
		human_suggested_loa_history_ = 0;
		ai_suggested_loa_history_ = 0;
	}
	
}