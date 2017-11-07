
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "manager_api/AlertManagement.h"

class Limits{

public:
  Limits(ros::NodeHandle& n)
  {
    // Read parameters
    n.param<float> ("min_range", params_["min_range"], 0.65);
    n.param<float> ("max_value_lin", params_["max_value_lin"], 1.0);
    n.param<float> ("min_value_lin", params_["min_value_lin"], -0.5);
    n.param<float> ("max_value_ang", params_["max_value_ang"], 1.7);
    n.param<float> ("max_delta", params_["max_delta"], 0.5);

    sub_vel_ = n.subscribe("/mux_vel_raw/cmd_vel",1000,&Limits::velCallback,this);

    sub_laser_ = n.subscribe("/laser_scan",1000,&Limits::laserCallback,this);

    manager.initPublisher(n);
  }

  // Stop robot movement
  void stop()
  {
    ROS_INFO("Stopping robot...");
    manager.warn("Stopping robot...");
    //ej, jestes oddzielnym komputerem... czemu nie mógłbyś przestać odbierać pakietów??
    std::string command = "iptables -I INPUT -j DROP";
    system(command.c_str());
  }

  //TODO: dodać ograniczenia przyśpieszenia -- zapisywać ostatnią wartość prędkości o sprawdzać deltę
  void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    float linear = msg->linear.x;
    float angular = msg->angular.z;

    if(linear>params_["max_value_lin"] || linear<params_["min_value_lin"])
    {
      ROS_INFO("Received incorrect linear velocity");
      stop();
    } 

    if(angular>params_["max_value_ang"] || angular<-params_["max_value_ang"])
    {
      ROS_INFO("Received incorrect angular velocity");
      stop();
    }
    // check acceleration
    if(!first_run)
    {
      if(abs(linear-last_linear) > params_["max_delta"] ||
          abs(angular-last_angular) > params_["max_delta"])
      {
        ROS_INFO("Received acceleration not valid");
        stop();
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
    for (auto r : ranges)
    {
      if(r<params_["min_range"]){
        ROS_INFO("Laser: unsave distance");
        stop();
      } 
    }

  }

private:
  // parameters
  std::map<std::string, float> params_;
  // last values
  float last_linear;
  float last_angular;
  bool first_run = true;

  ros::Subscriber sub_vel_;
  ros::Subscriber sub_laser_;
  manager_api::AlertManagement manager = manager_api::AlertManagement("limits");

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "limits");

  ros::NodeHandle n;
  Limits limits(n);
  ROS_INFO("Limits started!");

  ros::spin();
  return 0;
}
