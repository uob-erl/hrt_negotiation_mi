#include <ros/ros.h>
#include "negotiation_mi.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "negotiation_mi_node");

  ros::NodeHandle nh, private_nh("~");

  NegotiationMI test_object(nh, private_nh);
  ros::spin();
  // ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  // int rate;
  // string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  //     ros::NodeHandle private_node_handle_("~");
  //     private_node_handle_.param("rate", rate, int(40));
  //     private_node_handle_.param("topic", topic, string("example"));

  // Create a new NodeExample object.

  // // Tell ROS how fast to run this node.
  // ros::Rate r(10);

  // // Main loop.
  // while (n.ok())
  // {

  //   ros::spinOnce();
  //   r.sleep();
  // }

  return 0;

} // end main()
