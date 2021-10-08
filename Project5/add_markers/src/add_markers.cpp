#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

#include <cmath>

// Pickup and drop off coordinates
const double pickup_x = -2.0;
const double pickup_y = 5.0;

const double dropoff_x = -0.5;
const double dropoff_y = -3.0;

const double tolerance = 0.4;

#define TOLERANCE 0.4

bool picking_up = true;
bool dropping_off = false;

ros::Publisher marker_pub;
visualization_msgs::Marker marker;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

  // ROS_INFO("Odom received! Processing ...");

  float odom_x = msg->pose.pose.position.x;
  float odom_y = msg->pose.pose.position.y;

  float dx;
  float dy;
  float dist;

  if (picking_up) {


    dx = -odom_x - pickup_y; //invert value becacuse map is 90deg rotated
    dy = odom_y - pickup_x;
    dist = std::sqrt(dx*dx+dy*dy);
    ROS_INFO("Picking up dist = %f", dist);
    if (dist < tolerance && picking_up) {
      // we are at the pickup location
      ROS_INFO("Pick UP object");

      // Issue pickup-delete marker
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);

      // Wait 5 seconds before trying to show other markers
      ros::Duration(5.0).sleep();

      picking_up = false;
      dropping_off = true;
      return;
    }

  } else if (dropping_off) {


    dx = -odom_x - dropoff_y; //invert value becacuse map is 90deg rotated
    dy = odom_y - dropoff_x;
    dist = std::sqrt(dx*dx+dy*dy);
    ROS_INFO("Dropping off dist = %f", dist);

    if (dist < tolerance) {
      // we are at the pickup location
      ROS_INFO("Drop OFF object");

      marker.pose.position.x = dropoff_x;
      marker.pose.position.y = dropoff_y;

      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);

      ros::Duration(5.0).sleep();
      picking_up = false;
      dropping_off = false;
      return;
    }
  }

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;


  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pickup_x;
  marker.pose.position.y = pickup_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  ros::Duration(1.0).sleep();

  ROS_INFO("Adding marker ...");
  marker_pub.publish(marker);

  ros::Subscriber odom_sub = n.subscribe("/odom", 1, odomCallback);

  ros::Rate r(10.0);
  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

  return 0;

}
