#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

// Global variable indicating if the robot reached the pickup zone
bool reached_pickup = false;
// Global variable indicating the margin in odom pose position to consider we arrived at a location
double margin = 0.02;

/* Callback function called when new odom data is received */
void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
/*  if(msg->pose.pose.position.x == 2.0 && msg->pose.pose.position.y == 0.0 && msg->pose.pose.position.z == 0.0 && msg->pose.pose.orientation.x == 0.0 && msg->pose.pose.orientation.y == 0.0 && msg->pose.pose.orientation.z == 0.0 && msg->pose.pose.orientation.w == 1.0){*/
  double tx = msg->pose.pose.position.x;
  double ty = msg->pose.pose.position.y;
  double tz = msg->pose.pose.position.z;
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", tx, ty, tz);  
  ROS_INFO("Difference-> x: [%f], y: [%f]", fabs(tx - 2.0), fabs(ty));  
  if(fabs(tx - 2.0) < margin && fabs(ty) < margin && fabs(tz) < margin){
    ROS_INFO("Arrived at pickup location!");
    reached_pickup = true;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "cube");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("odom", 1000, callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  
  marker.header.frame_id = "/map"; 
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cube";
  marker.id = 0;

  // Set the marker type: cube  
  marker.type = shape;

  // Set the marker action.  
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  
  // This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 2;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  // The marker will stay visible until a newer marker is sent
  marker.lifetime = ros::Duration();

  // Publish the marker once the first subscriber subscribes to topic
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);

  ROS_INFO("Published first");

  // When the robot reaches the pickup location, the marker should disappear. 
  while(!reached_pickup) {
    ros::spin();
  }

  marker.header.stamp = ros::Time::now();

  // Set the marker action.  
  marker.action = visualization_msgs::Marker::DELETE;

  marker_pub.publish(marker);
  ROS_INFO("Marker disappeared");

  // Wait 5 sec to simulate pickup 
  ros::Duration(1).sleep();

  // Set the pose of the marker.  
  // This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 1;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = -1.0;

  // The market will stay indefinitely long
  marker.lifetime = ros::Duration();

  marker_pub.publish(marker);
  ROS_INFO("Published second");

  ros::spin();
}

