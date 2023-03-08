#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>
#include "tf/transform_datatypes.h"
#include "pipes.h"
#include <mutex>
#include <vector>
#include <atomic>
#include "gazebo_tf/SetGoals.h"
#include "gazebo_tf/GetMinDistToGoal.h"

class Reach
{
public:
Reach(ros::NodeHandle nh) :
    nh_(nh), goalSet_(false),goalReached_(false)
{

    ros::NodeHandle pnh("~"); // Create private node handle
    std::string source="ugv"; // Default Name
    pnh.getParam("platform", source);

    std::string topic_name = "/" + source + "_odom/";

    ROS_INFO_STREAM("The connection to ros for platform:" << source);
    ROS_INFO_STREAM("Topic name:" << topic_name);

    sub_ = nh_.subscribe(topic_name, 1000, &Reach::OdoCallback,this);

    serviceSetGoal_ = nh_.advertiseService(ros::this_node::getName() + "/set_goals", &Reach::setGoal,this);
    serviceCheckGoal_ = nh_.advertiseService(ros::this_node::getName() + "/check_goals", &Reach::checkGoals,this);

    minDist_=1e6;

}

~Reach(){

}

bool setGoal(gazebo_tf::SetGoals::Request  &req,
             gazebo_tf::SetGoals::Response &res)
{
  mx_.lock();
  goals_=req.goals;
  goalSet_=true;
  goalReached_.assign(req.goals.size(),false);
  goalIdx_=0;
  for (unsigned int i=0;i<req.goals.size();i++) {
      ROS_INFO_STREAM("Goal "<< i << " x,y ="<< goals_.at(i).x << " , " << goals_.at(i).y );
  }
  mx_.unlock();
  startTime_ = ros::Time::now();

  return true;
}

bool checkGoals(gazebo_tf::GetMinDistToGoal::Request  &req,
             gazebo_tf::GetMinDistToGoal::Response &res)
{

 ROS_INFO_STREAM("Checking Goals!!!!");
 mx_.lock();
 bool reached = true;
 for (unsigned int i=0;i<goals_.size();i++) {
     reached = reached & goalReached_.at(i);
     if(goalReached_.at(i)){
         ROS_INFO_STREAM("Goal:" << i << " Reached!");
     }
     else{
         ROS_INFO_STREAM("Goal:" << i << " NOT Reached!");
     }
 }
 mx_.unlock();
 res.reached = reached;
 res.dist=minDist_;

  return true;
}


void OdoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{


    if(goalSet_){
        mx_.lock();
        if(goalIdx_<goals_.size()){
            double dx = goals_.at(goalIdx_).x - msg->pose.pose.position.x ;
            double dy = goals_.at(goalIdx_).y - msg->pose.pose.position.y ;
            mx_.unlock();
            double d=std::pow( std::pow(dx,2) + std::pow(dy,2),0.5);

            ROS_INFO_STREAM_THROTTLE(5,"Goal:" << goalIdx_ << " dist:"<< d);
            if(d<1.0){
                ROS_INFO_STREAM("Reached Goal:" << goalIdx_ << " dist:"<< d);
                goalReached_.at(goalIdx_)=true;
                goalIdx_++;
            }
        }
        else{
          if(goalSet_){
               std::stringstream ss;
               ss << ros::this_node::getName() << " goals :" <<  goals_.size();
               bool reach=true;
               for (unsigned int i=0;i<goals_.size();i++) {
                   reach = reach & goalReached_.at(i);
                   ss << " G:" << i << " ";
                   if(goalReached_.at(i)){
                        ss << " OK ";
                   }
                   else {
                       ss << " NOT ";
                   }
               }
               ROS_INFO_STREAM(ss.str());
               if (reach){
                   ros::Time endTime = ros::Time::now();
                   double dt = endTime.toSec() - startTime_.toSec();
                   ROS_INFO_STREAM("GOALS REACHED : " <<  dt );
                   minDist_=dt;
              }
              goalSet_=false;
          }
        }
        mx_.unlock();
    }

}

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::atomic<bool> goalSet_;
    std::vector<bool> goalReached_;
    std::vector<geometry_msgs::Point> goals_;
    unsigned int goalIdx_;
    std::mutex mx_;
    ros::ServiceServer serviceSetGoal_;
    ros::ServiceServer serviceCheckGoal_;
    double minDist_;
    ros::Time startTime_;
};


/**
 * May need two threads to allow reconnecting if a timeout occurs (no comms)
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "reach");

  ros::NodeHandle nh;

  std::shared_ptr<Reach> reachPtr(new Reach(nh));

  ros::spin();

  ros::shutdown();


  return 0;
}
