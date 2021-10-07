
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

class AddMarker
{
private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber odom_sub;
  visualization_msgs::Marker marker;
  double init[2] = {-2.0, 5.0};
  double end[2] = {-0.5, -3.0};
  double delta;
  const int mul= 2;

public:
  AddMarker() {

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    odom_sub = n.subscribe("/odom", 1, &AddMarker::odomCallback, this);
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = init[0];
    marker.pose.position.y = init[1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    
    delta = marker.scale.x;

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      ROS_WARN("Please create a subscriber to the marker");
      sleep(1);
    }

    ROS_INFO("Subscriber successfully");
    marker_pub.publish(marker);
    
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    float x_pose             = msg->pose.pose.position.x;
    float y_pose            = msg->pose.pose.position.y;
    float x_scale           = marker.scale.x;
    float y_scale           = marker.scale.y;

    if ((-x_pose < (init[1] + x_scale)) && (-x_pose > (init[1] - x_scale)) && (y_pose < (init[0] + y_scale)) && (y_pose > (init[0] -y_scale))) {
      ROS_INFO("Goal reached");
      ros::Duration(0.5).sleep();

      marker.action = visualization_msgs::Marker::DELETE;
      ROS_INFO("Delete object");
      marker_pub.publish(marker);    
    }
      else if ((-x_pose < (end[1]+ 0.6)) && (-x_pose > (end[1] - 0.6)) && (y_pose < (end[0] + 0.6))) {
        ROS_INFO("Reached End Goal");
        ros::Duration(2).sleep();

        marker.pose.position.x = end[0];
        marker.pose.position.y = end[1];
        marker.action = visualization_msgs::Marker::ADD;

        marker_pub.publish(marker);
        
      }
    }
};


int main( int argc, char** argv )
{

  ros::init(argc, argv, "add_markers");
  
  AddMarker add_marker;
  ros::spin();

  return 0; 
}
