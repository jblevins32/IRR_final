#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <math>

using std::placeholders::_1;
using namespace std::chrono_literals;

// This node gets the odom topic and controls the robot to a defined location
class control : public rclcpp::Node
{
public:
    control():Node("get_control") // This is initializing the node
    {
      RCLCPP_INFO(this->get_logger(), "Starting Control Node");

      rclcpp::QoS qos = rclcpp::QoS(10).best_effort(); // Quality of Service

      subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom_fixed", qos, std::bind(&control::odom_callback, this, _1)); // _1 means the function will allow one argument
      
      subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", qos, std::bind(&control::scan_callback, this, _1)); // _1 means the function will allow one argument

      subscription_sign = this->create_subscription<int>(
        "/detected_sign", qos, std::bind(&control::sign_callback, this, _1));

      publisher_cmdvel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      timer_ = this->create_wall_timer(20ms, std::bind(&control::timer_callback, this));
    }

private:

    // Define sensor messages
    sensor_msgs::msg::LaserScan recent_scan;
    nav_msgs::msg::Odometry recent_odom;
    int recent_sign;
    bool odom_received = false;
    bool scan_received = false;
    bool sign_received = false;
    bool pause_flag = false;

    // Set linear and angular speeds
    float lin_vel = 0.1;
    float ang_vel = 0.35;

    // Instantiate a Twist for control command and defining desired distance
    geometry_msgs::msg::Twist twist_msg;

    // meters to stay from obstacles
    float obstacle_dist = 0.3;

    // Waypoint errors
    float error_threshold = 0.01;
    float error_threshold_rot = 0.05;

    // Get current yaw orientation
    double current_yaw = quaternionToYaw(recent_odom.pose.pose.orientation);

    // Distance between grids
    float grid_dist = 1; // meters

    // callback to capture recent scan
    void scan_callback(const sensor_msgs::msg::LaserScan & laser_msg) // message type & variable name
    {
        // RCLCPP_INFO(this->get_logger(), "Received Laser Scan");

        recent_scan = laser_msg;
        scan_received = true;
    }

    // callback to capture recent odom
    void odom_callback(const nav_msgs::msg::Odometry & odom_msg)
    {
        // RCLCPP_INFO(this->get_logger(),  "Received Odom");

        recent_odom = odom_msg;
        odom_received = true;
    }

    // callback to capture sign detections
    void sign_callback(const int & detected_sign)
    {
        // RCLCPP_INFO(this->get_logger(),  "Received Sign");

        recent_sign = detected_sign;
        odom_received = true;
    }

    void timer_callback()
      {
        // Ensure we have data from sensors
        if (scan_received && odom_received && sign_received)
        {
          // Get current pose from odom
          auto current_dist_x = recent_odom.pose.pose.position.x;
          auto current_dist_y = recent_odom.pose.pose.position.y;

          // Get current scan data in front of robot from odom
          auto current_scan = recent_scan.ranges;
          std::vector<float> current_scan_fov;

          // isolate the front 120 degrees of lidar scan
          // current_scan_fov.insert(current_scan_fov.end(), current_scan.begin() + 300, current_scan.end());
          // current_scan_fov.insert(current_scan_fov.end(), current_scan.begin(),current_scan.begin() + 60); 
          // current_scan_fov.insert(current_scan_fov.end(), current_scan.begin() + 190, current_scan.end());
          // current_scan_fov.insert(current_scan_fov.end(), current_scan.begin(),current_scan.begin() + 29); 

          // isolate the front of lidar scan
          current_scan_fov.insert(current_scan_fov.end(), current_scan.begin() + 355, current_scan.end());
          current_scan_fov.insert(current_scan_fov.end(), current_scan.begin(),current_scan.begin() + 5); 

          // Get min distance object in front of robot
          float scan_min = *std::min_element(current_scan_fov.begin(), current_scan_fov.end());

          // If an obstacle is less than some dist from the front of the robot,
          if (scan_min <= obstacle_dist)
          {
            // Read sign
            DoSignAction();
          }
          
          // No obstacle ahead, translate 1 grid
          else
          {
            translate(recent_odom + grid_dist)
          }
        }
      }

    // Do the action requested by the sign
    void DoSignAction()
    {
      // None, rotate 90 to look for another sign
      if (recent_sign == 0)
      {
        rotate(current_yaw + math::pi/2);
      } 

      // left arrow
      else if (recent_sign == 1)
      {
        rotate(current_yaw + math::pi/2);
      }

      // right arrow
      else if (recent_sign == 2)
      {
        rotate(current_yaw - math::pi/2);
      }

      // Stop and turn around
      else if (recent_sign == 3 | recent_sign == 4)
      {
        rotate(current_yaw - math::pi);
      }

      // Goal
      else if (recent_sign == 5)
      {
        RCLCPP_INFO(this->get_logger(), "Found goal!");
        stop()
      }
    }

    // Call for rotate commands until some desired yaw
    void rotate(double desired_yaw)
    {

      // Stop movement before rotation
      twist_msg.linear.x = 0;
      float yaw_error = getYawError(current_yaw, desired_yaw);

      // Rotate this direction until we meet the threshold
      while (yaw_error > error_threshold_rot)
      {
        twist_msg.angular.z = ang_vel;
        publish_cmd(twist_msg);
      }

      // Rotate this direction until we meet the threshold
      while (-yaw_error > error_threshold_rot)
      {
        twist_msg.angular.z = -ang_vel;
        publish_cmd(twist_msg);
      }
    }

    // Call for translate commands until some desired position
    void translate(double desired_x)
    {

      // Stop movement before translation
      twist_msg.angular.z = 0;
      float dist_error = recent_odom - desired_x

      // translate in x until we meet the threshold
      while (dist_error > error_threshold)
      {
        twist_msg.linear.x = lin_vel;
        publish_cmd(twist_msg);
      }
    }

    void stop()
    {
      // Stop movement
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0;
      publisher_cmdvel->publish(twist_msg);
    }

    // Publish a velocity command
    void publish_cmd(geometry_msgs::msg::Twist twist_msg)
    {
      publisher_cmdvel->publish(twist_msg);
    }

    // Convert quaternions to yaw
    double quaternionToYaw(const geometry_msgs::msg::Quaternion &orientation)
    {
      tf2::Quaternion tf2_quat;
      tf2::fromMsg(orientation, tf2_quat);

      double roll, pitch, yaw;
      tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);

      return yaw;
    }

    // Get error in yaw. Corrects complete rotation issue.
    float getYawError(float current_yaw, float desired_yaw)
    {
      float yaw_error = desired_yaw - current_yaw;
      if (yaw_error > M_PI)
      {
        yaw_error -= 2*M_PI;
      }
      else if (yaw_error < -M_PI)
      {
        yaw_error += 2*M_PI;
      }
      return yaw_error;
    }

    // memory management in cpp
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmdvel;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan;
    rclcpp::Subscription<int>::SharedPtr subscription_sign;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Setup ros2 system
    rclcpp::spin(std::make_shared<control>());
    rclcpp::shutdown();
    return 0;
};