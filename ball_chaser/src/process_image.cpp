#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


struct pixel_pointer
{
    uint row;
    uint col;
    uint len;
};

class SubscribeAndPublish
{
public:

    SubscribeAndPublish() // This is the constructor
    {
        
        // Define a client service capable of requesting services from command_robot
        client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

        // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
        subscriber_ = n_.subscribe("/camera/rgb/image_raw", 10, &SubscribeAndPublish::process_image_callback, this);

    }


    // This callback function continuously executes and reads the image data
    void process_image_callback(const sensor_msgs::Image img)
    {
        int white_pixel = 255;
        int rows = img.height;
        int cols = img.width;

        // The img.data is the width*heigth*3 1D array
        // I need to convert it to width*heigth 2D array = grayscale image
        uint row = 0;
        uint col = 0;
        uint grayscale[rows][cols];
        for (int i=0; i < rows*cols*3 - 2;i = i + 3)
        {
            row = (int) i / (cols*3);
            col = (int)(i % (cols*3))/3;
            grayscale[row][col] = (int)((img.data[i] + img.data[i+1] + img.data[i+2]) / 3);
        }

        uint max_hor = 0; // the longest line of white pixels horizontaly
        uint cur_hor = 0;
        pixel_pointer horizontal;

        // I want to find the largest horizontal and vertical lines on the image
        // If they intersect - that intersection will be the center of the white ball (sort of)
        for (int i=0; i < rows; i++)
        {
            for (int j=1; j < cols; j++)
            {
                if ((grayscale[i][j] == white_pixel) && (grayscale[i][j-1] == white_pixel))
                    cur_hor++;
                else
                {
                    if (cur_hor > max_hor)
                    {
                        max_hor = cur_hor;
                        horizontal.row = i;
                        horizontal.col = j;
                        horizontal.len = max_hor;
                    }
                    cur_hor = 0;
                }
            }
            if (cur_hor > max_hor)
            {
                max_hor = cur_hor;
                horizontal.row = i;
                horizontal.col = cols - 1;
                horizontal.len = max_hor;
            }
            cur_hor = 0;
        }
        pixel_pointer vertical;
        max_hor = 0;
        cur_hor = 0;
        for (int j=0; j < cols; j++)
        {
            for (int i=1; i < rows; i++)
            {
                if ((grayscale[i][j] == white_pixel) && (grayscale[i-1][j] == white_pixel))
                    cur_hor++;
                else
                {
                    if (cur_hor > max_hor)
                    {
                        max_hor = cur_hor;
                        vertical.row = i;
                        vertical.col = j;
                        vertical.len = max_hor;
                    }
                    cur_hor = 0;
                }
            }
            cur_hor = 0;
        }
        //printf("The longest horizontal line %d at %d %d\n", horizontal.len, horizontal.row, horizontal.col);
        //printf("The longest vertical line %d at %d %d\n", vertical.len, vertical.row, vertical.col);

        // Check for intersection (this would fit for a white square also, but ... oh well...)
        if ((vertical.col >= horizontal.col - horizontal.len) && (vertical.col <= horizontal.col) &&
            (horizontal.row >= vertical.row - vertical.len) && (horizontal.row <= vertical.row))
            {
                printf("Intersection at %d %d\n", horizontal.row, vertical.col);
                int shift = vertical.col - cols/2; // image shift from center
                int sign = copysign(1.0, shift); // double;
                sign = -sign;

                // And now we generate the motion command for our robot!
                if (horizontal.len > cols*0.7) // 70% gives the robot some time to slow down
                    drive_robot(0.0, 0.0); // if the ball is too close - stop
                else if (abs(shift) < cols*0.15)
                {
                    drive_robot(2.5, 0.0);
                    //printf("Moving forward!\n");
                }
                else if (abs(shift) < cols*0.4)
                {
                    drive_robot(0.3, sign*0.2);
                    //printf("Turning at %f\n", sign*0.2);
                }
                else
                {
                    drive_robot(0.3, sign*0.4);
                    //printf("Turning at %f\n", sign*0.4);
                }
            }
            else
                drive_robot(0.0, 0.0); // if the ball wasn't found - stop
    }

    // This function calls the command_robot service to drive the robot in the specified direction
    void drive_robot(float lin_x, float ang_z)
    {
      ball_chaser::DriveToTarget srv;
      srv.request.linear_x = lin_x;
      srv.request.angular_z = ang_z;

      // Call the safe_move service and pass the requested velocities
      client_.call(srv);
    }

private:
    // ROS services
    ros::NodeHandle n_;
    ros::Subscriber subscriber_;
    ros::ServiceClient client_;

};//End of class SubscribeAndPublish



int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");

    // Handle ROS communication events
    SubscribeAndPublish SAPObject;
        
    ros::spin();

    return 0;
}
