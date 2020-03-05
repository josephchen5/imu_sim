//example_ros_class.cpp:
//wsn, Jan 2016
//illustrates how to use classes to make ROS nodes
// constructor can do the initialization work, including setting up subscribers, publishers and services
// can use member variables to pass data from subscribers to other member functions

// can test this function manually with terminal commands, e.g. (in separate terminals):
// rosrun example_ros_class example_ros_class
// rostopic echo example_class_output_topic
// rostopic pub -r 4 example_class_input_topic std_msgs/Float32 2.0
// rosservice call example_minimal_service     // sends a trigger signal; don't need a request argument

// this header incorporates all the necessary #include files and defines the class "ExampleRosClass"
// #include "example_ros_class.h"
#include "imu_sim_class.h"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
ExampleRosClass::ExampleRosClass(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of ExampleRosClass");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    initializeServices();

    //initialize variables here, as needed
    val_to_remember_ = 0.0;

    // // IMU model
    Param params;
    IMU imuGen(params);

    // create imu data
    // imu pose gyro acc
    std::vector<MotionData> imudata;
    std::vector<MotionData> imudata_noise;

    int hz = 50;
    ros::Rate loop_rate(hz);

    for (float t = params.t_start; t < params.t_end;)
    {

        ROS_INFO(" Time : %lf S", t);

        MotionData data = imuGen.MotionModel(t);

        Eigen::Vector3d imu_position;
        Eigen::Matrix3d imu_rotation;
        imu_position = data.twb;
        imu_rotation = data.Rwb;

        Publishtf(tfb, imu_position, imu_rotation);

        imudata.push_back(data);
        PublishPath(imu_path_publisher_, imudata);

        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        imudata_noise.push_back(data_noise);

        t += 1.0 / params.imu_frequency;
        loop_rate.sleep();
    }

    // can also do tests/waits to make sure all required services, topics, etc are alive
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void ExampleRosClass::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &ExampleRosClass::subscriberCallback, this);
    // add more subscribers here, as needed
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void ExampleRosClass::initializeServices()
{
    ROS_INFO("Initializing Services");
    minimal_service_ = nh_.advertiseService("example_minimal_service",
                                            &ExampleRosClass::serviceCallback,
                                            this);
    // add more services here, as needed
}

//member helper function to set up publishers;
void ExampleRosClass::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    minimal_publisher_ = nh_.advertise<std_msgs::Float32>("example_class_output_topic", 1, true);
    imu_path_publisher_ = nh_.advertise<nav_msgs::Path>("imu_path", 1, true);

    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

// a simple callback function, used by the example subscriber.
// note, though, use of member variables and access to minimal_publisher_ (which is a member method)
void ExampleRosClass::subscriberCallback(const std_msgs::Float32 &message_holder)
{
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"

    val_from_subscriber_ = message_holder.data; // copy the received data into member variable, so ALL member funcs of ExampleRosClass can access it
    ROS_INFO("myCallback activated: received value %f", val_from_subscriber_);
    std_msgs::Float32 output_msg;
    val_to_remember_ += val_from_subscriber_; //can use a member variable to store values between calls; add incoming value each callback
    output_msg.data = val_to_remember_;
    // demo use of publisher--since publisher object is a member function
    minimal_publisher_.publish(output_msg); //output the current value of val_to_remember_
}

//member function implementation for a service callback function
bool ExampleRosClass::serviceCallback(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response)
{
    ROS_INFO("service callback activated");
    response.success = true; // boring, but valid response info
    response.message = "here is a response string";
    return true;
}

void ExampleRosClass::Publishtf(tf::TransformBroadcaster &tfb, Eigen::Vector3d &position, Eigen::Matrix3d &rotation)
{
    // 參考 https://github.com/ros-planning/navigation/blob/kinetic-devel/amcl/src/amcl_node.cpp#L1359

    std::string new_base_frame_id_ = "/base_link";
    std::string global_frame_id_ = "/map";

    // tf::Transform new_tf(tf::createQuaternionFromYaw(finalPose(2)),
    //                      tf::Vector3(finalPose(0), finalPose(1), 0.0));

    tf::Quaternion q;
    q.setRPY(0, 0, 0);

    tf::Transform new_tf(q, tf::Vector3(position(0), position(1), position(2)));

    tf::StampedTransform new_tf_stamped(new_tf,
                                        ros::Time::now(),
                                        global_frame_id_, new_base_frame_id_);
    tfb.sendTransform(new_tf_stamped);
}

void ExampleRosClass::PublishPath(ros::Publisher &puber, std::vector<MotionData> &imudata)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "/map";

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/map";

    std::cout << " numbers: = " << imudata.size() << std::endl;

    for (int i = 0; i < imudata.size(); i++)
    {
        Eigen::Vector3d traj_node = imudata[i].twb;
        pose.pose.position.x = traj_node(0);
        pose.pose.position.y = traj_node(1);
        pose.pose.position.z = traj_node(2);

        tf::Quaternion q;
        q.setRPY(0, 0, 0);

        pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        path_msg.poses.push_back(pose);
    }

    puber.publish(path_msg);
}

int main(int argc, char **argv)
{
    // ROS set-ups:
    ros::init(argc, argv, "gener_alldata_node"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type ExampleRosClass");
    ExampleRosClass exampleRosClass(&nh); //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    // int hz = 20;
    // ros::Rate loop_rate(hz);

    // for (float t = params.t_start; t < params.t_end;)
    // {

    //     ROS_INFO(" Time : %lf S", t);

    //     MotionData data = imuGen.MotionModel(t);

    //     Eigen::Vector3d imu_positiontf;
    //     imu_positiontf = data.twb;

    //     Publishtf(tfb, imu_positiontf);

    //     imudata.push_back(data);

    //     // add imu noise
    //     MotionData data_noise = data;
    //     imuGen.addIMUnoise(data_noise);
    //     imudata_noise.push_back(data_noise);

    //     t += 1.0 / params.imu_frequency;

    //     loop_rate.sleep();
    // }

    ROS_INFO("main: going into spin; let the callbacks do all the work");

    ros::spin();
    return 0;
}
