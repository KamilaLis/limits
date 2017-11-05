
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

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

    diagnostic_pub_ = n.advertise<diagnostic_msgs::DiagnosticStatus>("limits/info",1);
  }

  // Send info to diagnostics topic
  void sendDiagnosticMsg(const std::string& msg, int level)
  {
      diagnostic_msgs::DiagnosticStatus message;
      message.level = level;
      message.name = "IDS:robot";
      message.message = msg.c_str();
      diagnostic_msgs::KeyValue values;
      values.key = "rosTime";
      values.value = std::to_string(ros::Time::now().toSec());
      message.values = {values};
      diagnostic_pub_.publish(message);
  }

  // Stop robot movement
  void stop()
  {
    ROS_INFO("Stopping robot...");
    this->sendDiagnosticMsg("Stopping robot...", 1);
    //ej, jestes oddzielnym komputerem... czemu nie mógłbyś przestać odbierać pakietów??
    //coś jak ochrona przed zalaniem, blokuję przyjmowanie
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

  ros::Subscriber sub_vel_;
  ros::Subscriber sub_laser_;
  ros::Publisher diagnostic_pub_;

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
