#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Joy.h"
#include <visualization_msgs/MarkerArray.h>

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>


class UGVRemote
{
public:
UGVRemote(ros::NodeHandle nh) :
    nh_(nh),
    steering_ratio_(17.3),
    lock_to_lock_revs_(3.2),
    max_steer_angle_(M_PI*lock_to_lock_revs_/steering_ratio_),
    max_break_torque_(8000.0)
{
    throttle_pub = nh_.advertise<std_msgs::Float64>("/orange/throttle_cmd", 10, false);
    brake_pub = nh_.advertise<std_msgs::Float64>("/orange/brake_cmd", 10, false);
    steering_pub = nh_.advertise<std_msgs::Float64>("/orange/steering_cmd", 10, false);
    sub_ = nh_.subscribe("/joy", 100, &UGVRemote::joyCallback,this);
}

~UGVRemote(){

}


void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

    ROS_INFO_STREAM( "seq steering brake/throttle " <<  msg->header.seq << " "
                   << msg->axes.at(1) << " " << msg->axes.at(3) );

//axes: [0.03420161083340645, -0.0, 0.0, -0.0, -0.0, 0.0]
//axes: [-0.0, -0.0, 0.0, 0.14271900057792664, -1.0, 0.0]



//    ROS_DEBUG_STREAM( "seq brake/steering/throttle " <<  ugv.seq << " "
//               << ugv.brake << " " << ugv.steering << " " << ugv.throttle);

    std_msgs::Float64 msg_brake, msg_throttle, msg_steering;

    if(msg->axes.at(0)>=0){

        msg_throttle.data = msg->axes.at(1);//Already goes to 1.0 max!
        throttle_pub.publish(msg_throttle);
    }
    else{
        msg_brake.data = -1.0*(msg->axes.at(1))*max_break_torque_;
        brake_pub.publish(msg_brake);
    }


    msg_steering.data = msg->axes.at(3)*max_steer_angle_*steering_ratio_;
    steering_pub.publish(msg_steering);

}

private:
    ros::NodeHandle nh_;
    ros::Publisher throttle_pub ,brake_pub ,steering_pub;
    ros::Subscriber sub_;
    const double steering_ratio_;
    const double lock_to_lock_revs_;
    const double max_steer_angle_;
    const double max_break_torque_;

};


/**
 * May need two threads to allow reconnecting if a timeout occurs (no comms)
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ugv_remote");

  ros::NodeHandle nh;
  UGVRemote ugvRemote(nh);

  ros::spin();

  ros::shutdown();

  return 0;
}
