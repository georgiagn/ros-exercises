#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

// Pickup pose parameters
float pickup_position_x = 3.0;
float pickup_position_y = 0.0;
float pickup_position_z = 0.0;
float pickup_orientation_x = 0.0;
float pickup_orientation_y = 0.0;
float pickup_orientation_z = 0.0;
float pickup_orientation_w = 1.0;

// Dropoff pose parameters
float dropoff_position_x = 1.0;
float dropoff_position_y = 0.0;
float dropoff_position_z = 0.0;
float dropoff_orientation_x = 0.0;
float dropoff_orientation_y = 0.0;
float dropoff_orientation_z = 0.0;
float dropoff_orientation_w = -1.0;

// Global variable indicating if the robot reached the pickup zone
bool reached_pickup = false;
// Global variable indicating if the robot reached the dropoff zone
bool reached_dropoff = false;
// Global variable indicating the margin in odom pose position to consider we arrived at a location
double margin = 0.2;

/* Callback function called when new odom data is received */
void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double tx = msg->pose.pose.position.x;
  double ty = msg->pose.pose.position.y;
  double tz = msg->pose.pose.position.z;
  if(fabs(tx - pickup_position_x) < margin && fabs(ty - pickup_position_y) < margin && fabs(tz - pickup_position_z) < margin){
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", tx, ty, tz);  
    ROS_INFO("Difference-> x: [%f], y: [%f]", fabs(tx - pickup_position_x), fabs(ty - pickup_position_y));  
    ROS_INFO("Arrived at pickup location!");
    reached_pickup = true;
  }
 
  else if(fabs(tx - dropoff_position_x) < margin && fabs(ty - dropoff_position_y) < margin && fabs(tz - dropoff_position_z) < margin){
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", tx, ty, tz);  
    ROS_INFO("Difference-> x: [%f], y: [%f]", fabs(tx - dropoff_position_x), fabs(ty - dropoff_position_y));  
    ROS_INFO("Arrived at dropoff location!");
    reached_dropoff = true;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "cube");
  ros::NodeHandle n;
  ros::Rate r(100); // 100 Hz
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("odom", 1000, callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp 
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
  marker.pose.position.x = pickup_position_x;
  marker.pose.position.y = pickup_position_y;
  marker.pose.position.z = pickup_position_z;
  marker.pose.orientation.x = pickup_orientation_x;
  marker.pose.orientation.y = pickup_orientation_y;
  marker.pose.orientation.z = pickup_orientation_z;
  marker.pose.orientation.w = pickup_orientation_w;

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
  // We poll the global variable reached_pickup at 100 Hz frequency.
  while(ros::ok){
    ros::spinOnce();
    if(reached_pickup)
      break;
    r.sleep();
  }

  // Set the marker action and new timestamp 
  marker.action = visualization_msgs::Marker::DELETE;
  marker.header.stamp = ros::Time::now();

  marker_pub.publish(marker);
  ROS_INFO("Marker disappeared");

  // Wait 5 sec to simulate pickup 
  ros::Duration(5).sleep();

  // Set the pose of the marker.  
  // This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = dropoff_position_x;
  marker.pose.position.y = dropoff_position_y;
  marker.pose.position.z = dropoff_position_z;
  marker.pose.orientation.x = dropoff_orientation_x;
  marker.pose.orientation.y = dropoff_orientation_y;
  marker.pose.orientation.z = dropoff_orientation_z;
  marker.pose.orientation.w = dropoff_orientation_w;

  // The marker at the dropoff location will stay visible until a newer marker is sent
  // marker.lifetime = ros::Duration();
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker_pub.publish(marker);
  ROS_INFO("Published second");

  // When the robot reaches the dropoff location, the marker should disappear. 
  // We poll the global variable reached_dropoff at 100 Hz frequency.
  reached_dropoff = false;
  while(ros::ok){
    ros::spinOnce();
    if(reached_dropoff)
      break;
    r.sleep();
  }

  // Set the marker action and new timestamp 
  marker.action = visualization_msgs::Marker::DELETE;
  marker.header.stamp = ros::Time::now();

  marker_pub.publish(marker);
  ROS_INFO("Marker disappeared");

  sleep(10);
}

