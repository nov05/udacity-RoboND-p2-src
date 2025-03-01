#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    // Request to drive my robot forward
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    // https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    // int white_pixel = 255;
    bool ball_detected = false;

    int left_count = 0;
    int center_count = 0;
    int right_count = 0;

    int count = 0;
    for (size_t i = 0; i < img.height * img.step; i += 3)
    {
        int red = img.data[i];
        int green = img.data[i + 1];
        int blue = img.data[i + 2];

        // Next check if you found the white color ball
        if (red == 1 && green == 1 && blue == 1)
            count += 1;
    }
    ROS_INFO("yellow pixels detected: %d", count)

    if (ball_detected == true)
        drive_robot(0.5, 0);
}

int main(int argc, char **argv)
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
