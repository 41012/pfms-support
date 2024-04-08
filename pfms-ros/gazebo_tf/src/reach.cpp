#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include <sstream>
// thread and chrono are for time and sleeping respectively
#include <chrono>
#include <thread>
#include <string>
// #include "tf/transform_datatypes.h"

#include <mutex>
#include <vector>
#include <atomic>
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

// #include "gazebo_tf/GetMinDistToGoal.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Reach : public rclcpp::Node
{
public:
Reach() :
    Node("reach_node"), 
    goalSet_(false),goalReached_(false),dt_(0), dStart_(0), dTravelled_(0)
{

    //! @todo: change to use parameters for distance
    distanceThreshold_ = 0.7;

    sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/orange/odom", 1000, std::bind(&Reach::odoCallback,this,_1));

    sub2_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/orange/check_goals", 1000, std::bind(&Reach::goalsCallback,this,_1));

    serviceCheckGoals_ = this->create_service<std_srvs::srv::Trigger>("ackerman_check_goals", 
                                std::bind(&Reach::checkGoals,this, std::placeholders::_1, std::placeholders::_2));

    // std::string my_param = this->get_parameter("robot_state_publisher/robot_description").as_string();
    // RCLCPP_INFO(this->get_logger(), "Robot description: %s", my_param.c_str());

}

~Reach(){

}

// void setGoals( const std::shared_ptr<pfms_interfaces::srv::SetGoals::Request> request,
//           std::shared_ptr<pfms_interfaces::srv::SetGoals::Response>    response)
void goalsCallback(const geometry_msgs::msg::PoseArray& msg)
{
   mx_.lock();
   goals_=msg;
   goalSet_=true;
   goalReached_.assign(goals_.poses.size(),false);
   goalDist_.assign(goals_.poses.size(),1000.0);

   goalIdx_=0;int i=0;
   for (auto goal : goals_.poses) {
       RCLCPP_INFO_STREAM(this->get_logger(),"Goal "<< i++ << " x,y ="<< goal.position.x << " , " << goal.position.y );
   }
   mx_.unlock();
   dStart_ = dTravelled_;
   startTime_ = this->get_clock()->now();}

void checkGoals(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response>    response)
{

    RCLCPP_INFO_STREAM(this->get_logger(),"Checking Goals within " << distanceThreshold_ << "[m]");
    mx_.lock();
    bool success = true;
    std::stringstream msg;

    if(goals_.poses.size()==0){
        RCLCPP_INFO(this->get_logger(),"No goals set!");
        success = false;
    }
    else{
        for (unsigned int i=0;i<goals_.poses.size();i++) {
            success = success & goalReached_.at(i);
            msg << goalDist_.at(i) << ",";
            RCLCPP_INFO_STREAM(this->get_logger(),"Goal:" << i << " "  << goalDist_.at(i) << "[m]");
        }
    }
    
    if (success) {
        msg << dt_;
    }
    else {
        msg << "0";
    }
    mx_.unlock();
    response->success = success;
    response->message = msg.str();
}

void odoCallback(const nav_msgs::msg::Odometry& msg)
{

    double d = pow (pow(msg.pose.pose.position.x-odoPrev_.pose.pose.position.x,2) +
                        pow(msg.pose.pose.position.y-odoPrev_.pose.pose.position.y,2),0.5);
    std::atomic<double> dAtomic(d); // complications because we have one variable that is atomic (dTravelled) hence need to do this
    dTravelled_= dTravelled_ + dAtomic;
    odoPrev_=msg;

        if(goalSet_){
        mx_.lock();
        if(goalIdx_<goals_.poses.size()){
            auto goal = goals_.poses.at(goalIdx_);
            geometry_msgs::msg::Pose goalPrev;
            if(goalIdx_>0){
                goalPrev = goals_.poses.at(goalIdx_-1);            
            }
            mx_.unlock();
            
            double d = distToGoal(msg,goal);
            RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(),
                *this->get_clock(),
                1000,
                "Goal [id,d]=[" << goalIdx_ << ","<< d << "]  odo:" << dTravelled_-dStart_ ) ;

            if(d<distanceThreshold_){
                RCLCPP_INFO_STREAM(this->get_logger(),"Reached Goal:" << goalIdx_ << " dist:"<< d);
                goalReached_.at(goalIdx_)=true;
                goalDist_.at(goalIdx_)=d;
                goalIdx_++;
            }

            //Let's check previous goal as we could still be rolling towards it
            if(goalIdx_>0){
                double dPrev = distToGoal(msg,goalPrev);
                if(dPrev<goalDist_.at(goalIdx_-1)){
                    // RCLCPP_INFO_STREAM(this->get_logger(),"Reached Goal:" << goalIdx_-1 << " dist:"<< dPrev);
                    goalDist_.at(goalIdx_-1)=dPrev;
                }
            }

            if(goalIdx_==goals_.poses.size()){
                rclcpp::Duration dt = this->get_clock()->now() - startTime_;
                dt_ =  dt.seconds();
                RCLCPP_INFO_STREAM(this->get_logger(),"All goals reached in " <<  dt_ <<"[s] " << dTravelled_-dStart_ << "[m]" );                
                goalSet_ = false;
            }
        }
        mx_.unlock();
    }

}

double distToGoal(const nav_msgs::msg::Odometry& msg, const geometry_msgs::msg::Pose& goal)
{
    double dx = goal.position.x - msg.pose.pose.position.x ;
    double dy = goal.position.y - msg.pose.pose.position.y ;
    return std::pow( std::pow(dx,2) + std::pow(dy,2),0.5);
}

private:

    //ros::Subscriber sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub2_;

    std::atomic<bool> goalSet_;
    std::vector<bool> goalReached_;
    std::vector<double> goalDist_;
    geometry_msgs::msg::PoseArray goals_;

    unsigned int goalIdx_;
    std::mutex mx_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr serviceCheckGoals_;
    
    rclcpp::Time startTime_;
    double dt_;
    double dStart_;
    std::atomic<double> dTravelled_;
    nav_msgs::msg::Odometry odoPrev_;
    double distanceThreshold_;
};


int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Reach>());
  rclcpp::shutdown();

  return 0;
}
