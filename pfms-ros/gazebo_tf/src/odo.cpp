#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>
#include "tf/transform_datatypes.h"
#include <mutex>
#include <atomic>
#include "gazebo_tf/SetGoal.h"
#include "gazebo_tf/GetMinDistToGoal.h"

class Odo
{
public:
Odo(ros::NodeHandle nh) :
    nh_(nh), goalSet_(false),goalReached_(false)
{

    ros::NodeHandle pnh("~"); // Create private node handle
    std::string source="ugv"; // Default Name
    pnh.getParam("platform", source);

    std::string topic_name = "/" + source + "_odom/";

    ROS_INFO_STREAM("The connection to ros for platform:" << source);
    ROS_INFO_STREAM("Topic name:" << topic_name);

    sub_ = nh_.subscribe(topic_name, 1000, &Odo::odoCallback,this);

    serviceSetGoal_ = nh_.advertiseService("set_goal", &Odo::setGoal,this);
    serviceCheckGoal_ = nh_.advertiseService("check_goal", &Odo::checkGoal,this);

}

~Odo(){
}

bool setGoal(gazebo_tf::SetGoal::Request  &req,
             gazebo_tf::SetGoal::Response &res)
{
  mx_.lock();
  goal_=req.goal;
  goalSet_=true;
  goalReached_=false;
  minDist_=1e6;
  mx_.unlock();
  ROS_INFO_STREAM("Goal x,y ="<< goal_.x << " , " << goal_.y );

  return true;
}

bool checkGoal(gazebo_tf::GetMinDistToGoal::Request  &req,
             gazebo_tf::GetMinDistToGoal::Response &res)
{
  mx_.lock();
  res.dist=minDist_;
  goalSet_=false;
  mx_.unlock();
  ROS_INFO_STREAM("Min dist to goal:" << minDist_);

  return true;
}


void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    if(goalSet_){
      double d=std::pow( std::pow(msg->pose.pose.position.x-goal_.x,2) +
                         std::pow(msg->pose.pose.position.y-goal_.y,2),0.5);

//      ROS_INFO_STREAM("d:"<< d);
      if(d<minDist_){
        minDist_=d;
      }

      if(d<0.7){
        goalReached_=true;
        goalSet_=false;
        ROS_INFO_STREAM("Reached goal d:"<< d);
      }
    }

}

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::atomic<bool> goalSet_;
    std::atomic<bool> goalReached_;
    geometry_msgs::Point goal_;
    std::mutex mx_;
    ros::ServiceServer serviceSetGoal_;
    ros::ServiceServer serviceCheckGoal_;
    double minDist_;
};


/**
 * May need two threads to allow reconnecting if a timeout occurs (no comms)
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "odo");

  ros::NodeHandle nh;

  std::shared_ptr<Odo> odo(new Odo(nh));

  ros::spin();

  ros::shutdown();


  return 0;
}
