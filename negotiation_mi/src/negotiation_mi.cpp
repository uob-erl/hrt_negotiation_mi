#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>

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
        human_loa_sub_ = nh_.subscribe("/human_suggested_loa", 5, &NegotiationMI::humanLoaCallback, this);
        ai_loa_sub_ = nh_.subscribe("/ai_suggested_loa", 5, &NegotiationMI::humanLoaCallback, this);

        // Publishers AKA ROS node outputs
        negotiated_loa_pub_ = nh_.advertise<std_msgs::Int8>("/negotiated_loa", 1);

        current_loa_.data = 0; // initializing to "stop" mode
        human_suggested_loa_ = 0;
        ai_suggested_loa_ = 0;
        loa_changed_ = false;
}

NegotiationMI::~NegotiationMI()
{
}

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

// This reads and stored the up-to-date LOA suggested by human/operator
void NegotiationMI::humanLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
        human_suggested_loa_ = msg->data;
}

// This reads and stored the up-to-date LOA suggested by the AI
void NegotiationMI::aiLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
        ai_suggested_loa_ = msg->data;
}

// The main negotiation algorithm were magic happens
int NegotiationMI::negotiationAlgorithm()
{

        // QUESTIONS:
        // 1 - when negotiation algorithm starts? Everytime there is a LOA switch different than current LOA?

        if (human_changed_LOA)
        {
         while (deadline > time_since_negatioation_started)
             {wait for AI input
              }
         if (ai_input_given)
         { start negotiation //update a flag 
         }
         else 
         publish loa and leave negotiation algorithm


        }

        else if (ai_changed_loa)
        {
                while (deadline > time_since_negatioation_started)
             {wait for human input
              }
         if (human_input_given)
         { start negotiation //update a flag 
         }
         else 
         publish loa and leave negotiation algorithm
        }

        If (negotiation_is_active)
        {
                start negotiating
        }

                // this can istead be publish the resul in ROS t
                return something;
}