#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
float last_linear_x = 0;
float last_angular_z = 0;
const float speed = 0.5;
const float angle = 0.3;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float linear_x, float angular_z)
{
    std::string node_name = ros::this_node::getName();

    // TODO: Request a service and pass the velocities to it to drive the robot
    // Request to drive my robot forward
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;

    if (linear_x != last_linear_x || angular_z != last_angular_z)
    {
        // Make the service call
        if (!client.call(srv))
            ROS_ERROR("%s: Failed to call service drive_to_target", node_name.c_str());
    }

    last_linear_x = linear_x;
    last_angular_z = angular_z;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    // https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    float linear_x = 0;
    float angular_z = 0;
    int left_count = 0;
    int center_count = 0;
    int right_count = 0;
    int ball_detect_shreshold = 1000;

    for (size_t i = 0; i < img.height * img.step; i += 3)
    {
        int red = img.data[i];
        int green = img.data[i + 1];
        int blue = img.data[i + 2];

        // Calculate the pixel's x-coordinate (column)
        int column = (i / 3) % img.width;

        // Next check if you found the yellow color pixel
        if (red == 255 && green == 255 && blue == 0)
        {
            if (column < img.width / 3)
            {
                left_count += 1;
            }
            else if (column >= (img.width / 3) && column < (img.width - img.width / 3))
            {
                center_count += 1;
            }
            else
            {
                right_count += 1;
            }
        }
    }

    int total_count = left_count + center_count + right_count;
    // If it is close to the ball, stop
    if (total_count > 60000 || total_count < 100)
    {
        linear_x = 0;
        angular_z = 0;
    }
    // If the ball is in the left area, turn left and move forward
    else if (left_count >= center_count && left_count >= right_count)
    {

        linear_x = speed;
        angular_z = angle;
    }
    // If the ball is in the center area, move forward
    else if (center_count > left_count && center_count >= right_count)
    {
        linear_x = speed;
        angular_z = 0;
    }
    // If the ball is in the right area, turn right and move forward
    else if (right_count > center_count && right_count > left_count)
    {
        linear_x = speed;
        angular_z = -angle;
    }
    // Or stop
    else
    {
        linear_x = 0;
        angular_z = 0;
    }

    drive_robot(linear_x, angular_z);
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
