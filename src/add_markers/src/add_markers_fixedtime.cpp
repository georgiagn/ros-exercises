#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

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

int main( int argc, char** argv )
{
  ros::init(argc, argv, "cube");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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

  // The marker will stay visible for 5 sec
  marker.lifetime = ros::Duration(5);

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
  ROS_INFO("Published pickup marker");

  // Wait for a total of 10 sec (5 sec for marker to disappear and 5 sec to simulate pickup)
  ros::Duration(10).sleep();

  // Set the pose of the dropoff marker.  
  // This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = dropoff_position_x;
  marker.pose.position.y = dropoff_position_y;
  marker.pose.position.z = dropoff_position_z;
  marker.pose.orientation.x = dropoff_orientation_x;
  marker.pose.orientation.y = dropoff_orientation_y;
  marker.pose.orientation.z = dropoff_orientation_z;
  marker.pose.orientation.w = dropoff_orientation_w;

  // Set the marker action, new timestamp and infinite display time (or until a new marker is sent)
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration();

  marker_pub.publish(marker);
  ROS_INFO("Published dropoff marker");

  ros::spin();
}

