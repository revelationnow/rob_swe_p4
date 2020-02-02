#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

float prev_error = 0.0;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // TODO: Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  if(!client.call(srv))
  {
    ROS_ERROR("Couldn't call");
  }
}



// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

  int white_pixel = 255;

  // TODO: Loop through each pixel in the image and check if there's a bright white one
  // Then, identify if this pixel falls in the left, mid, or right side of the image
  // Depending on the white ball position, call the drive_bot function and pass velocities to it
  // Request a stop when there's no white ball seen by the camera
  float lin_x = 0;
  float ang_z = 0;

  std::vector<std::vector<int> > img_data;

  for(int i = 0; i < img.height; i++)
  {
    std::vector<int> temp;
    for(int j = 0; j < img.step; j++)
    {
      temp.push_back(img.data[(i * img.step + j)]);
    }
    img_data.push_back(temp);
  }

  long long int sum_x_coord = 0;
  long long int num_white_pixels = 0;
  for(int i = 0; i < img.height * img.step; i++)
  {
    if(img.data[i] == white_pixel)
    {
      int x_coord = i % img.step;
      sum_x_coord += x_coord;
      num_white_pixels++;
    }
  }

  if(num_white_pixels > 0)
  {
    float average_x_coord = sum_x_coord/num_white_pixels;
    ROS_INFO("X_coord : %f, Step : %d",average_x_coord, img.step);

    // Proportional controller
    float error = (img.step - 2*average_x_coord);
    float P = 2;
    ang_z = P * ( error * M_PI)/(4 * img.step);

    //Differential Controller
    float change_in_error = (prev_error - error)/img.step;
    //ang_z = change_in_error * ang_z;

    // Slow down when angle is large
    lin_x = std::max(0.1, 1 - (fabs((2 * ang_z)/M_PI)));
    ROS_INFO("error : %f, prev_error : %f, lin_x : %f, ang_z : %f",error, prev_error, lin_x, ang_z);

    prev_error = error;
  }

  drive_robot(lin_x, ang_z);
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
