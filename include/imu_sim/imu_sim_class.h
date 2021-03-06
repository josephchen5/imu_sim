// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times

// #ifndef IMU_SIM_CLASS_H_
// #define IMU_SIM_CLASS_H_

#pragma once

#include <ros/ros.h> //ALWAYS need to include this
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Path.h"

#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"


//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>


#include "Eigen/Core"
#include "eigen3/Eigen/Geometry"

// #include "eigen3/Eigen/Geometry"


#include <fstream>
#include <sys/stat.h>
#include "imu.h"
#include "utilities.h"

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h> // uses the "Trigger.srv" message defined in ROS

// define a class, including a constructor, member variables and member functions
class ExampleRosClass
{
public:
    ExampleRosClass(ros::NodeHandle *nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::ServiceServer minimal_service_;
    ros::Publisher minimal_publisher_;

    tf::TransformBroadcaster tfb;

    ros::Publisher imu_path_publisher_;
    ros::Publisher imu_noise_path_publisher_;


    double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    double val_to_remember_;     // member variables will retain their values even as callbacks come and go

    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();

    void subscriberCallback(const std_msgs::Float32 &message_holder); //prototype for callback of example subscriber
    //prototype for callback for example service
    bool serviceCallback(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response);

    void Publishtf(tf::TransformBroadcaster &tfb, Eigen::Vector3d &position, Eigen::Matrix3d &rotation);

    void PublishPath(ros::Publisher &puber, std::vector<MotionData> &imudata);

}; // note: a class definition requires a semicolon at the end of the definition

// #endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
