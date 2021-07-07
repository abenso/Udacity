#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// The image will be divided in 3 parts, center_perc determine the size in percentage of the pixel considerate like center. This must be lest than 100
const int center_perc = 50;

// Percentage of pixels found in the center of image to stop the robot
const int cover_center_perc = 20;

// Percentage of pixels found in the center related to the center part
const float stop_perc = float(center_perc * cover_center_perc) / 10000.0;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
	ball_chaser::DriveToTarget srv;
	
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");

}

std::vector<float> get_maximum_speed(void)
{
    float max_linear_speed, max_angular_speed;

    // Assign a new node handle since we have no access to the main one
    ros::NodeHandle n2;
    // Get node name
    std::string node_name = ros::this_node::getName();
    // Get joints min and max parameters
    n2.getParam(node_name + "/max_linear_speed", max_linear_speed);
    n2.getParam(node_name + "/max_angular_speed", max_angular_speed);

    // Store clamped joint angles in a clamped_data vector
    std::vector<float> ret = { max_linear_speed, max_angular_speed };

    return ret;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
	int c_left, c_center, c_right;
    int steps_center, steps_side, stop_cont;
	float speed_v, speed_w;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	
    // Divide the image in three parts and then count in each part the numbers of white points

	// Define the number of pixels in each part
	steps_center = int(img.width*center_perc/100);
	steps_side = int(img.width*(100 - center_perc)/2/100);

	// Count the number of white pixels in each part
	c_left = 0;
	c_center = 0;
	c_right = 0;
    for (int i = 0; i < img.height; i++) {
		for(int j = 0; j < img.width; j++) {
			if(img.data[i*img.step + j*3] == white_pixel && img.data[i*img.step + j*3 + 1] == white_pixel && img.data[i*img.step + j*3 + 2] == white_pixel) {
				if (j < steps_side) {
					c_left++;
				} else {
					if (j < steps_side + steps_center) {
						c_center++;
					} else {
						c_right++;
					}
				}
			}
		}
    }

	// calc number of white pixels to stop the robot
	stop_cont = img.data.size() * stop_perc / 3;	

	// get maximun speeds from Parameter Server
	std::vector<float> max_speed = get_maximum_speed();

	// Speeds logic
	speed_w = 0.0;
	speed_v = 0.0;
	
	// when there are white pixels in both sides, stop angular speed 
	if (c_left != 0 && c_right != 0) {
		speed_w = 0.0;
	} else {
		if(c_left != 0) {
			speed_w = max_speed[1];
		} 
		if (c_right != 0) {
			speed_w = -max_speed[1];
		} 
	}

	if (c_center != 0) {
		speed_v = max_speed[0];
		//if number of pixels in the center is greater than limit, stop the robot
		if (c_center >= stop_cont) {
			speed_v = 0.0;
			speed_w = 0.0;
		}
	}

	// send speeds 
	drive_robot(speed_v, speed_w);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
