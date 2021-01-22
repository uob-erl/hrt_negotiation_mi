
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/package.h>
#include <string.h>

class StatusPublisher
{
public:
        StatusPublisher();

private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Publisher loa_pub_, nav_status_pub_;
        image_transport::Publisher neg_auto_pub_, neg_teleop_pub_;
        ros::Publisher deadline_pub_; /// conventional publisher not explicitly for ImgageTransport: problem?
        ros::Subscriber loa_sub_, nav_status_sub_, nav_result_sub_;
        ros::Subscriber negotiation_status_sub_, negotiation_time_sub_;
        ros::Subscriber negotiated_loa_sub_, human_suggested_loa_sub_, ai_suggested_loa_sub_;
        ros::Timer timerPubStatus_;

        int nav_status_, nav_result_;
        // ROS msg images
        sensor_msgs::Image rosImgAuto_, rosImgTeleop_, rosImgStop_, rosImgCanceled_;
        sensor_msgs::Image rosImgActive_, rosImgSucceeded_, rosImgAborted_;
        sensor_msgs::Image rosImgTeleopDis_, rosImgTeleopEn_, rosImgTeleopAi_, rosImgTeleopHuman_, rosImgTeleopNegLoa_;
        sensor_msgs::Image rosImgAutoDis_, rosImgAutoEn_, rosImgAutoAi_, rosImgAutoHuman_, rosImgAutoNegLoa_;

        void loaCallBack(const std_msgs::Int8::ConstPtr &loa);
        void nav_statusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &nav_status);
        void nav_resultCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr &nav_result);
        void humanLoaCallback(const std_msgs::Int8::ConstPtr &msg);
        void aiLoaCallback(const std_msgs::Int8::ConstPtr &msg);
        void negStatusCallback(const std_msgs::Bool::ConstPtr &msg);
        void negLoaCallback(const std_msgs::Int8::ConstPtr &msg);
        void deadlineCallBack(const std_msgs::Float64::ConstPtr &msg);
        void timerPubStatusCallback(const ros::TimerEvent &);
};

// Constractor
StatusPublisher::StatusPublisher() : it_(nh_)
{
        // Initialise status/result values

        nav_status_ = -1;
        nav_result_ = -1;

        // Subscribers
        loa_sub_ = nh_.subscribe<std_msgs::Int8>("/loa", 1, &StatusPublisher::loaCallBack, this);
        nav_status_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1,
                                                                         &StatusPublisher::nav_statusCallBack, this);
        nav_result_sub_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1,
                                                                              &StatusPublisher::nav_resultCallBack, this);
        negotiation_time_sub_ = nh_.subscribe<std_msgs::Float64>("/nemi/negotiation_time", 1, &StatusPublisher::deadlineCallBack, this);
        human_suggested_loa_sub_ = nh_.subscribe("/human_suggested_loa", 5, &StatusPublisher::humanLoaCallback, this);
        ai_suggested_loa_sub_ = nh_.subscribe("/ai_suggested_loa", 5, &StatusPublisher::aiLoaCallback, this);

        // publishers
        loa_pub_ = it_.advertise("/robot_status/loa", 1, true);
        nav_status_pub_ = it_.advertise("/robot_status/nav", 1, true);
        deadline_pub_ = nh_.advertise<sensor_msgs::Image>("/nemi/hmi_neg_deadline", 1);
        // neg_Auti_pub_ = it_.advertise("/robot_status/nav", 1, true);
        neg_auto_pub_ = it_.advertise("/nemi/hmi_auto", 1, true);
        neg_teleop_pub_ = it_.advertise("/nemi/hmi_teleop", 1, true);
        timerPubStatus_ = nh_.createTimer(ros::Duration(0.100), &StatusPublisher::timerPubStatusCallback, this);

        // temporary path variables
        std::string pathTeleop_, pathAuto_, pathStop_, pathCanceled_;
        std::string pathActive_, pathSucceeded_, pathAborted_;
        std::string pathTeleopDis_, pathTeleopEn_, pathTeleopAi_, pathTeleopHuman_, pathTeleopNegLoa_;
        std::string pathAutoDis_, pathAutoEn_, pathAutoAi_, pathAutoHuman_, pathAutoNegLoa_;

        // Path where the images are
        pathTeleop_ = ros::package::getPath("status_publisher");
        pathTeleop_.append("/images/teleop.png");

        pathAuto_ = ros::package::getPath("status_publisher");
        pathAuto_.append("/images/auto.png");

        pathStop_ = ros::package::getPath("status_publisher");
        pathStop_.append("/images/stop.png");

        pathActive_ = ros::package::getPath("status_publisher");
        pathActive_.append("/images/active.png");

        pathSucceeded_ = ros::package::getPath("status_publisher");
        pathSucceeded_.append("/images/succeeded.png");

        pathAborted_ = ros::package::getPath("status_publisher");
        pathAborted_.append("/images/aborted.png");

        pathCanceled_ = ros::package::getPath("status_publisher");
        pathCanceled_.append("/images/canceled.png");

        pathTeleopDis_ = ros::package::getPath("status_publisher");
        pathTeleopDis_.append("/images/teleop_gray.png");

        pathTeleopEn_ = ros::package::getPath("status_publisher");
        pathTeleopEn_.append("/images/teleop_lightblue.png");

        pathTeleopAi_ = ros::package::getPath("status_publisher");
        pathTeleopAi_.append("/images/teleop_blue.png");

        pathTeleopHuman_ = ros::package::getPath("status_publisher");
        pathTeleopHuman_.append("/images/teleop_orange.png");

        pathTeleopNegLoa_ = ros::package::getPath("status_publisher");
        pathTeleopNegLoa_.append("/images/teleop_green.png");

        pathAutoDis_ = ros::package::getPath("status_publisher");
        pathAutoDis_.append("/images/auto_gray.png");

        pathAutoEn_ = ros::package::getPath("status_publisher");
        pathAutoEn_.append("/images/auto_lightblue.png");

        pathAutoAi_ = ros::package::getPath("status_publisher");
        pathAutoAi_.append("/images/auto_blue.png");

        pathAutoHuman_ = ros::package::getPath("status_publisher");
        pathAutoHuman_.append("/images/auto_orange.png");

        pathAutoNegLoa_ = ros::package::getPath("status_publisher");
        pathAutoNegLoa_.append("/images/auto_green.png");

        // Safety Check if actually the image is there and loaded
        if (cv::imread(pathTeleop_.c_str()).empty())
                ROS_FATAL("Teleop image was not loaded. Could not be found on %s", pathTeleop_.c_str());

        else if (cv::imread(pathAuto_.c_str()).empty())
                ROS_FATAL("Auto image was not loaded. Could not be found on %s", pathAuto_.c_str());

        else if (cv::imread(pathStop_.c_str()).empty())
                ROS_FATAL("Stop image was not loaded. Could not be found on %s", pathStop_.c_str());

        else if (cv::imread(pathActive_.c_str()).empty())
                ROS_FATAL("Active image was not loaded. Could not be found on %s", pathActive_.c_str());

        else if (cv::imread(pathSucceeded_.c_str()).empty())
                ROS_FATAL("Succeeded image was not loaded. Could not be found on %s", pathSucceeded_.c_str());

        else if (cv::imread(pathAborted_.c_str()).empty())
                ROS_FATAL("Aborted image was not loaded. Could not be found on %s", pathAborted_.c_str());

        else if (cv::imread(pathCanceled_.c_str()).empty())
                ROS_FATAL("Goal cancel image was not loaded. Could not be found on %s", pathCanceled_.c_str());

        else if (cv::imread(pathTeleopDis_.c_str()).empty())
                ROS_FATAL("Teleop Dis image was not loaded. Could not be found on %s", pathTeleopDis_.c_str());

        else if (cv::imread(pathTeleopEn_.c_str()).empty())
                ROS_FATAL("Teleop En image was not loaded. Could not be found on %s", pathTeleopEn_.c_str());

        else if (cv::imread(pathTeleopAi_.c_str()).empty())
                ROS_FATAL("Teleop Ai image was not loaded. Could not be found on %s", pathTeleopAi_.c_str());

        else if (cv::imread(pathTeleopHuman_.c_str()).empty())
                ROS_FATAL("Teleop Human image was not loaded. Could not be found on %s", pathTeleopHuman_.c_str());

        else if (cv::imread(pathTeleopNegLoa_.c_str()).empty())
                ROS_FATAL("Teleop Neg Loa image was not loaded. Could not be found on %s", pathTeleopNegLoa_.c_str());

        else if (cv::imread(pathAutoDis_.c_str()).empty())
                ROS_FATAL("Auto Dis image was not loaded. Could not be found on %s", pathAutoDis_.c_str());

        else if (cv::imread(pathAutoEn_.c_str()).empty())
                ROS_FATAL("Auto En image was not loaded. Could not be found on %s", pathAutoEn_.c_str());

        else if (cv::imread(pathAutoAi_.c_str()).empty())
                ROS_FATAL("Auto Ai image was not loaded. Could not be found on %s", pathAutoAi_.c_str());

        else if (cv::imread(pathAutoHuman_.c_str()).empty())
                ROS_FATAL("Auto Human image was not loaded. Could not be found on %s", pathAutoHuman_.c_str());

        else if (cv::imread(pathAutoNegLoa_.c_str()).empty())
                ROS_FATAL("Auto Neg Loa image was not loaded. Could not be found on %s", pathAutoNegLoa_.c_str());

        else
                ROS_INFO("All images loaded successfully");

        // temporary cv_bridge images
        cv_bridge::CvImage cvAuto_, cvTeleop_, cvStop_, cvCanceled_;
        cv_bridge::CvImage cvActive_, cvSucceeded_, cvAborted_;
        cv_bridge::CvImage cvTeleopDis_, cvTeleopEn_, cvTeleopAi_, cvTeleopHuman_, cvTeleopNegLoa_;
        cv_bridge::CvImage cvAutoDis_, cvAutoEn_, cvAutoAi_, cvAutoHuman_, cvAutoNegLoa_;

        // Load auto image with openCv
        cvAuto_.image = cv::imread(pathAuto_.c_str());
        cvAuto_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load teleop image with openCv
        cvTeleop_.image = cv::imread(pathTeleop_.c_str());
        cvTeleop_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Stop image with openCv
        cvStop_.image = cv::imread(pathStop_.c_str());
        cvStop_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Active image with openCv
        cvActive_.image = cv::imread(pathActive_.c_str());
        cvActive_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Succeeded image with openCv
        cvSucceeded_.image = cv::imread(pathSucceeded_.c_str());
        cvSucceeded_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Aborted image with openCv
        cvAborted_.image = cv::imread(pathAborted_.c_str());
        cvAborted_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Cancel goal image with openCv
        cvCanceled_.image = cv::imread(pathCanceled_.c_str());
        cvCanceled_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Teleop Dis image with openCv
        cvTeleopDis_.image = cv::imread(pathTeleopDis_.c_str());
        cvTeleopDis_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Teleop En image with openCv
        cvTeleopEn_.image = cv::imread(pathTeleopEn_.c_str());
        cvTeleopEn_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Teleop Ai image with openCv
        cvTeleopAi_.image = cv::imread(pathTeleopAi_.c_str());
        cvTeleopAi_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Teleop Human image with openCv
        cvTeleopHuman_.image = cv::imread(pathTeleopHuman_.c_str());
        cvTeleopHuman_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load Teleop Neg Loa image with openCv
        cvTeleopNegLoa_.image = cv::imread(pathTeleopNegLoa_.c_str());
        cvTeleopNegLoa_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load auto Dis image with openCv
        cvAutoDis_.image = cv::imread(pathAutoDis_.c_str());
        cvAutoDis_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load auto En image with openCv
        cvAutoEn_.image = cv::imread(pathAutoEn_.c_str());
        cvAutoEn_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load auto Ai image with openCv
        cvAutoAi_.image = cv::imread(pathAutoAi_.c_str());
        cvAutoAi_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load auto Human image with openCv
        cvAutoHuman_.image = cv::imread(pathAutoHuman_.c_str());
        cvAutoHuman_.encoding = sensor_msgs::image_encodings::BGR8;
        // Load auto Neg Loa image with openCv
        cvAutoNegLoa_.image = cv::imread(pathAutoNegLoa_.c_str());
        cvAutoNegLoa_.encoding = sensor_msgs::image_encodings::BGR8;

        // convert to ROS image type
        cvAuto_.toImageMsg(rosImgAuto_);
        cvTeleop_.toImageMsg(rosImgTeleop_);
        cvStop_.toImageMsg(rosImgStop_);
        cvActive_.toImageMsg(rosImgActive_);
        cvSucceeded_.toImageMsg(rosImgSucceeded_);
        cvAborted_.toImageMsg(rosImgAborted_);
        cvCanceled_.toImageMsg(rosImgCanceled_);
        cvTeleopDis_.toImageMsg(rosImgTeleopDis_);
        cvTeleopEn_.toImageMsg(rosImgTeleopEn_);
        cvTeleopAi_.toImageMsg(rosImgTeleopAi_);
        cvTeleopHuman_.toImageMsg(rosImgTeleopHuman_);
        cvTeleopNegLoa_.toImageMsg(rosImgTeleopNegLoa_);
        cvAutoDis_.toImageMsg(rosImgAutoDis_);
        cvAutoEn_.toImageMsg(rosImgAutoEn_);
        cvAutoAi_.toImageMsg(rosImgAutoAi_);
        cvAutoHuman_.toImageMsg(rosImgAutoHuman_);
        cvAutoNegLoa_.toImageMsg(rosImgAutoNegLoa_);

        // Publish the default mode
        loa_pub_.publish(rosImgStop_);
        nav_status_pub_.publish(rosImgStop_);
        neg_auto_pub_.publish(rosImgAutoNegLoa_); //assuming simulation starts in autonomy
        neg_teleop_pub_.publish(rosImgTeleopDis_);
}

// takes care of loa publising in rviz
void StatusPublisher::loaCallBack(const std_msgs::Int8::ConstPtr &mode)
{
        if (mode->data == 0)
                loa_pub_.publish(rosImgStop_);
        if (mode->data == 1)
                loa_pub_.publish(rosImgTeleop_);
        if (mode->data == 2)
                loa_pub_.publish(rosImgAuto_);
}

// takes care of NAVIGATION current STATUS
void StatusPublisher::nav_statusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &nav_status)
{
        if (!nav_status->status_list.empty()) //First make you the vector is not empty to avoid memory allocation errors.
        {

                if (nav_status->status_list.size() > 1)
                {
                        actionlib_msgs::GoalStatus goalStatus = nav_status->status_list[1];
                        nav_status_ = goalStatus.status;
                }
                else
                {
                        actionlib_msgs::GoalStatus goalStatus = nav_status->status_list[0];
                        nav_status_ = goalStatus.status;
                }
        }
}

// takes care of NAVIGATION end Result
void StatusPublisher::nav_resultCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr &nav_result)
{
        nav_result_ = nav_result->status.status;
}

// displays human negotiation inputs
void StatusPublisher::humanLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
        if (msg->data == 1)
        {
                neg_auto_pub_.publish(rosImgAutoHuman_);
                neg_teleop_pub_.publish(rosImgTeleopAi_);
        }
        else if (msg->data == 2)
        {
                neg_teleop_pub_.publish(rosImgTeleopHuman_);
                neg_auto_pub_.publish(rosImgAutoAi_);
        }
}

// displays ai negotiation inputs
void StatusPublisher::aiLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
        if (msg->data == 1)
        {
                neg_auto_pub_.publish(rosImgAutoAi_);
        }
        else if (msg->data == 2)
        {
                neg_teleop_pub_.publish(rosImgTeleopAi_);
        }
}

// displays enabled & disabled negotiation interface
void StatusPublisher::negStatusCallback(const std_msgs::Bool::ConstPtr &msg)
{
        if (msg->data)
        {
                neg_auto_pub_.publish(rosImgAutoEn_);
                neg_teleop_pub_.publish(rosImgTeleopEn_);
        }
}

// displays negotiated = chosen LOA
void StatusPublisher::negLoaCallback(const std_msgs::Int8::ConstPtr &msg)
{
        if (msg->data == 1)
        {
                neg_auto_pub_.publish(rosImgAutoNegLoa_);
                neg_teleop_pub_.publish(rosImgTeleopDis_);
        }
        else if (msg->data == 2)
        {
                neg_teleop_pub_.publish(rosImgTeleopNegLoa_);
                neg_auto_pub_.publish(rosImgAutoDis_);
        }
        /// currently negotiated shown till new negotiation is enabled -> problem?
}

// Publishes deadline visualization
void StatusPublisher::deadlineCallBack(const std_msgs::Float64::ConstPtr &msg)
{  
        double time_norm = msg->data;
        int w = 100;
        int h = 25;
        // initialize empty cv matrix as the image container
        cv::Mat image = cv::Mat::zeros(h, w, CV_8UC3);

        // draw deadline bargraph if normalized negotiation time is within [0,1]
        if (time_norm >= 0)
        {
                // draw red rectangle, other parts of image are black by default initilization
                cv::rectangle(image,
                          cv::Point((int)w * time_norm, 0), // top left corner of rectangle in image
                          cv::Point(w, h),                  // bottom right corner of rectangle in image
                          cv::Scalar(255, 0, 0),            // red color
                          cv::FILLED,                       // FILLED equivalent to -1
                          cv::LINE_8);
        }
        // draw gray rectangle if negotiation is not running ie. time_norm = -1
        else
        {
                cv::rectangle(image,
                          cv::Point(0, 0),           // top left corner of rectangle in image
                          cv::Point(w, h),           // bottom right corner of rectangle in image
                         cv::Scalar(128, 128, 128), // gray color
                         cv::FILLED,                // FILLED equivalent to -1
                          cv::LINE_8);
        }

        // convert matrix Mat image to cvImage
        std_msgs::Header header;         // empty header
        header.seq = 0;                  // user defined counter
        header.stamp = ros::Time::now(); // time
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
        // convert cvImage to ROS message
        sensor_msgs::Image img_msg;     // message to be sent
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        // publish ROS msg
        deadline_pub_.publish(img_msg);
}

// The timer function that fuses everyting and publishes to the interface
void StatusPublisher::timerPubStatusCallback(const ros::TimerEvent &)
{
        // Publish current/running state
        if (nav_status_ == 1)
                nav_status_pub_.publish(rosImgActive_);

        // Publish end state of goals

        else if (nav_result_ == 3)
        {
                nav_status_pub_.publish(rosImgSucceeded_);
                nav_result_ = -1;
        }

        else if (nav_result_ == 2)
        {
                nav_status_pub_.publish(rosImgCanceled_);
                nav_result_ = -1;
        }

        else if (nav_result_ == 4)
        {
                nav_status_pub_.publish(rosImgAborted_);
                nav_result_ = -1;
        }

        else
        {
                //ROS_INFO("Status Something else?? Check /move_base/status for code");
        }
}

int main(int argc, char **argv)
{

        ros::init(argc, argv, "status_publisher");

        StatusPublisher publishStatus;

        ros::Rate loop_rate(10);

        // The main Loop where everything is runing

        while (ros::ok())
        {
                ros::spinOnce();
                loop_rate.sleep();
        }

        return 0;
}
