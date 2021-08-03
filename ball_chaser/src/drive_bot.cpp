#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"


// int, double to String converter
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}


class SubscribeAndPublish
{
public:

    SubscribeAndPublish() // This is the constructor
    {
        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
        service_ = n_.advertiseService("/ball_chaser/command_robot", &SubscribeAndPublish::handle_service_request, this);
    }


    // Handle_drive_request callback function that executes whenever a drive_bot service is requested
    // This function should publish the requested linear x and angular velocities to the robot wheel joints
    // After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
    bool handle_service_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
    {
        geometry_msgs::Twist joints_cmd;
        joints_cmd.linear.x = req.linear_x;
        joints_cmd.angular.z = req.angular_z;
        // Send motor commands
        publisher_.publish(joints_cmd);

        // Wait for a bit
        ros::Duration(1).sleep();

        // Return a response message
        res.linear_x_out = joints_cmd.linear.x;
        res.angular_z_out = joints_cmd.angular.z;

        return true;
    }

private:
    // ROS services
    ros::NodeHandle n_;
    ros::Publisher publisher_;
    ros::ServiceServer service_;
  
};//End of class SubscribeAndPublish



int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Handle ROS communication events
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}
