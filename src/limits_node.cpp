
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
//#include <diagnostic_msgs/DiagnosticStatus.h>
#include "manager_api/AlertManagement.h"
#include "manager_api/ManagerClient.h"

class Limits{

public:
  Limits(ros::NodeHandle& n)
  {
    // Read parameters
    n.param<float> ("min_range_forward", params_["min_range_forward"], 0.65);
    n.param<float> ("min_range_backward", params_["min_range_backward"], 0.65);
    n.param<float> ("max_value_lin", params_["max_value_lin"], 1.0);
    n.param<float> ("min_value_lin", params_["min_value_lin"], -0.5);
    n.param<float> ("max_value_ang", params_["max_value_ang"], 1.7);
    n.param<float> ("max_delta", params_["max_delta"], 0.5);

    n.getParam("velocity_topic", velocity_topic);
    n.getParam("laser_topic", laser_topic);

    sub_vel_ = n.subscribe(velocity_topic,1000,&Limits::velCallback,this);

    sub_laser_ = n.subscribe(laser_topic,1000,&Limits::laserCallback,this);

    manager_pub.initPublisher(n);
    manager_client.initManagerClient(n);
  }

  // Stop robot movement
  void stop(const std::string &msg)
  {
    ROS_INFO("Stopping robot...");
    manager_api::Message request = manager_api::Message::killSubsriber;
    manager_pub.error(msg, request, velocity_topic);
    // ask elektron_ids to kill subscriber of velocity_topic
    bool result = manager_client.error(msg, request, velocity_topic);
    if(!result)
    { // calling service failed
      ROS_INFO("Shutting down");
      std::string command = "shutdown now";
      system(command.c_str());
    }
  }


  void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    float linear = msg->linear.x;
    float angular = msg->angular.z;

    if(linear>params_["max_value_lin"] || linear<params_["min_value_lin"])
    {
      ROS_INFO("Received incorrect linear velocity");
      stop("Received incorrect linear velocity");
    } 

    if(angular>params_["max_value_ang"] || angular<-params_["max_value_ang"])
    {
      ROS_INFO("Received incorrect angular velocity");
      stop("Received incorrect angular velocity");
    }
    // check acceleration
    if(!first_run)
    {
      if(abs(linear-last_linear) > params_["max_delta"] ||
          abs(angular-last_angular) > params_["max_delta"])
      {
        ROS_INFO("Received acceleration not valid");
        stop("Received acceleration not valid");
      }
    }
    else
    {
      first_run = false;
    }
    // save last values
    last_linear = linear;
    last_angular = angular;

  }


  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    std::vector<float> ranges = msg->ranges;
    // have ranges changed? = is robot moving?
    if (last_ranges_ != ranges)
    { // check if robot is running forward or backward
      std::string range = last_linear>0 ? "min_range_forward" : "min_range_backward";
      for (auto r : ranges)
      { 
        if(r<params_[range]){
          ROS_INFO("Laser: unsave distance");
          stop("Laser: unsave distance");
        } 
      }
      // save ranges
      last_ranges_ = ranges;
    }
  }

private:
  // parameters
  std::map<std::string, float> params_;
  std::string velocity_topic, laser_topic;
  // last values
  float last_linear;
  float last_angular;
  bool first_run = true;
  std::vector<float> last_ranges_;

  bool stop_sended = false;

  ros::Subscriber sub_vel_;
  ros::Subscriber sub_laser_;
  manager_api::AlertManagement manager_pub = manager_api::AlertManagement("limits");
  manager_api::ManagerClient manager_client = manager_api::ManagerClient("limits");
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "limits");

  ros::NodeHandle n("~");
  Limits limits(n);
  ROS_INFO("Limits started!");

  ros::spin();
  return 0;
}
