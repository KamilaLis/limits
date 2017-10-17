
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticStatus.h>


// Send info to diagnostics topic
void ComponentIDS::sendDiagnosticMsg(const std::string& msg, int level)
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
}


void velCallback(const geometry_msgs::Twist::ConstPtr& msg,
                 float max_value_lin,
                 float min_value_lin,
                 float max_value_ang,
                 float max_delta)
{
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  if(linear>max_value_lin || linear<min_value_lin)
  {
    ROS_INFO("Received incorrect linear velocity");
    stop();
  } 

  if(angular>max_value_ang || angular<-max_value_ang)
  {
    ROS_INFO("Received incorrect angular velocity");
    stop();
  }
    
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg,
                   float min_range)
{
  std::vector<float> ranges = msg->ranges;
  for (auto r : ranges)
  {
    if(r<min_range) stop();
  }

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "limits");

  ros::NodeHandle n;
  // Read parameters
  float min_range, max_value_lin, min_value_lin, max_value_ang, max_delta;
  n.param<float> ("min_range", min_range, 1.2);
  n.param<float> ("max_value_lin", max_value_lin, 1.0);
  n.param<float> ("min_value_lin", min_value_lin, -0.5);
  n.param<float> ("max_value_ang", max_value_ang, 1.7);
  n.param<float> ("max_delta", max_delta, 0.5);

  ros::Subscriber sub_vel_ = n.subscribe<geometry_msgs::Twist>
                                    ("/mux_vel_raw/cmd_vel", 
                                    1000,
                                    boost::bind(velCallback, _1, max_value_lin,min_value_lin,max_value_ang, max_delta));
  ros::Subscriber sub_laser_ = n.subscribe<sensor_msgs::LaserScan>
                                    ("/laser_scan", 
                                    1000,
                                    boost::bind(laserCallback, _1, min_range));

  ros::Publisher diagnostic_pub_ = n.advertise<diagnostic_msgs::DiagnosticStatus>("limits/info",1);

  ROS_INFO("Limits started!");
  ros::spin();
  return 0;
}
