#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <tuple>

using std::placeholders::_1;
using namespace std::chrono_literals;

using MessageTriple = std::tuple<
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::LaserScan,
    std_msgs::msg::Int32>;

// This node gets the odom topic and controls the robot to a defined location
class control : public rclcpp::Node
{
public:
    control():Node("get_control") // This is initializing the node
    {
      RCLCPP_INFO(this->get_logger(), "Starting Control Node");

      rclcpp::QoS qos(rclcpp::KeepLast(10));
      qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
      
      subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom_fixed", qos, std::bind(&control::odom_callback, this, _1)); // _1 means the function will allow one argument
      
      subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", qos, std::bind(&control::scan_callback, this, _1)); // _1 means the function will allow one argument

      subscription_sign = this->create_subscription<std_msgs::msg::Int32>(
        "/detected_sign", qos, std::bind(&control::sign_callback, this, _1));

      publisher_cmdvel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      timer_ = this->create_wall_timer(20ms, std::bind(&control::timer_callback, this));
    }

private:

    bool odom_received = false;
    bool scan_received = false;
    bool sign_received = false;
    bool pause_flag = false;
    bool twisting = false;
    double current_yaw;

    // Keep track of when turning
    double initial_yaw;
    bool entered_rotate = false;
    float yaw_error;

    // Define sensor messages
    sensor_msgs::msg::LaserScan recent_scan;
    nav_msgs::msg::Odometry recent_odom;
    std_msgs::msg::Int32 recent_sign;

    // Set linear and angular speeds
    float lin_vel = 0.1;
    float ang_vel = 0.35;

    // Instantiate a Twist for control command and defining desired distance
    geometry_msgs::msg::Twist twist_msg;

    // meters to stay from obstacles
    float obstacle_dist = 0.6;

    // Waypoint errors
    float error_threshold = 0.01;
    float error_threshold_rot = 0.05;

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
    }

    // callback to capture sign detections
    void sign_callback(const std_msgs::msg::Int32 & detected_sign)
    {
        // RCLCPP_INFO(this->get_logger(),  "Received Sign");

        recent_sign = detected_sign;
    }

    void timer_callback()
      {

        // Wait until we get the slowest topic, lidar scan
        if (scan_received) 
        {
          // RCLCPP_INFO(this->get_logger(), "Received all");

          // Get current pose from odom
          auto current_dist_x = recent_odom.pose.pose.position.x;

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

          // std::ostringstream oss;
          // for (size_t i = 0; i < current_scan_fov.size(); ++i) {
          //   oss << current_scan_fov[i];
          //   if (i != current_scan_fov.size() - 1)
          //     oss << ", ";
          // }
          // RCLCPP_INFO(this->get_logger(), "Scan: [%s]", oss.str().c_str());

          // Get min distance object in front of robot
          float scan_min = *std::min_element(current_scan_fov.begin(), current_scan_fov.end());
          RCLCPP_INFO(this->get_logger(), "Scan_min: [%f]", scan_min);

          // Get current yaw orientation
          current_yaw = quaternionToYaw(recent_odom.pose.pose.orientation);

          // If an obstacle is less than some dist from the front of the robot,
          if (scan_min <= obstacle_dist)
          {
            // Read sign
            
            // None, rotate 90 to look for another sign
            if (recent_sign.data == 0)
            {
              RCLCPP_INFO(this->get_logger(), "Found nothing");
              if (!entered_rotate)
              {
                initial_yaw = current_yaw;
              }
              
              rotate(initial_yaw + M_PI/2);
            } 

            // left arrow
            else if (recent_sign.data == 1)
            {
              RCLCPP_INFO(this->get_logger(), "Found left arrow");
              rotate(current_yaw + M_PI/2);
            }

            // right arrow
            else if (recent_sign.data == 2)
            {
              RCLCPP_INFO(this->get_logger(), "Found right arrow");
              rotate(current_yaw - M_PI/2);
            }

            // Stop and turn around
            else if ((recent_sign.data == 3) | (recent_sign.data == 4))
            {
              RCLCPP_INFO(this->get_logger(), "Found reverse or stop");
              rotate(current_yaw - M_PI);
            }

            // Goal
            else if (recent_sign.data == 5)
            {
              RCLCPP_INFO(this->get_logger(), "Found goal!");
              stop();
            }
          }
          }
          
          // No obstacle ahead, translate 1 grid
          else
          {
            translate(current_dist_x + grid_dist);
          }
        }
      }

    // Do the action requested by the sign
    void DoSignAction()
    {

    // Call for rotate commands until some desired yaw
    void rotate(double desired_yaw)
    {

      // Stop movement before rotation
      twist_msg.linear.x = 0;
      yaw_error = getYawError(current_yaw, desired_yaw);
      RCLCPP_INFO(this->get_logger(), "Current Yaw %f, desired Yaw %f", current_yaw, desired_yaw);
      // const auto &pos = recent_odom.pose.pose.position;
      // const auto &ori = recent_odom.pose.pose.orientation;
      // RCLCPP_INFO(this->get_logger(),
      //   "Odom Position: [x=%.2f, y=%.2f, z=%.2f]", pos.x, pos.y, pos.z);

      // RCLCPP_INFO(this->get_logger(),
      //   "Odom Orientation (quaternion): [x=%.2f, y=%.2f, z=%.2f, w=%.2f]", ori.x, ori.y, ori.z, ori.w);

      // Rotate this direction until we meet the threshold
      if (yaw_error > error_threshold_rot)
      {
        RCLCPP_INFO(this->get_logger(), "Rotating Clockwise");
        twist_msg.angular.z = ang_vel;
        publish_cmd(twist_msg);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Done Rotating");
        stop();
      }

      // Rotate this direction until we meet the threshold
      if (-yaw_error > error_threshold_rot)
      {
        RCLCPP_INFO(this->get_logger(), "Rotating CounterClockwise");
        twist_msg.angular.z = -ang_vel;
        publish_cmd(twist_msg);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Done Rotating");
        stop();
      }
    }

    // Call for translate commands until some desired position
    void translate(double desired_x)
    {

      // Stop movement before translation
      twist_msg.angular.z = 0;
      float dist_error = recent_odom.pose.pose.position.x - desired_x;

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
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_sign;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Setup ros2 system
    rclcpp::spin(std::make_shared<control>());
    rclcpp::shutdown();
    return 0;
};