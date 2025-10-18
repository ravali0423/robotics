#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class SMARTmBOTNode : public rclcpp::Node
{
  public:
    SMARTmBOTNode()
    : Node("smartmbot_node")
    {
      // Publishers
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      status_pub_ = this->create_publisher<std_msgs::msg::String>("smartmbot_status", 10);
      
      // Subscribers
      scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&SMARTmBOTNode::scan_callback, this, _1));
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&SMARTmBOTNode::odom_callback, this, _1));
      
      // Timer for robot behavior
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SMARTmBOTNode::timer_callback, this));
      
      RCLCPP_INFO(this->get_logger(), "SMARTmBOT node initialized!");
    }

  private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      // Basic obstacle avoidance logic
      if (!msg->ranges.empty()) {
        // Check front region for obstacles
        size_t front_start = msg->ranges.size() * 0.45;  // -10 degrees
        size_t front_end = msg->ranges.size() * 0.55;    // +10 degrees
        
        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = front_start; i < front_end && i < msg->ranges.size(); ++i) {
          if (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
            min_distance = std::min(min_distance, static_cast<double>(msg->ranges[i]));
          }
        }
        
        obstacle_detected_ = (min_distance < 1.0);  // 1 meter threshold
      }
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      current_odom_ = *msg;
    }
    
    void timer_callback()
    {
      auto twist_msg = geometry_msgs::msg::Twist();
      auto status_msg = std_msgs::msg::String();
      
      if (obstacle_detected_) {
        // Stop and turn if obstacle detected
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.5;  // Turn right
        status_msg.data = "Obstacle detected - turning";
      } else {
        // Move forward if no obstacles
        twist_msg.linear.x = 0.2;   // 0.2 m/s forward
        twist_msg.angular.z = 0.0;
        status_msg.data = "Moving forward";
      }
      
      cmd_vel_pub_->publish(twist_msg);
      status_pub_->publish(status_msg);
      
      // Reset obstacle detection flag
      obstacle_detected_ = false;
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Subscribers  
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State variables
    bool obstacle_detected_ = false;
    nav_msgs::msg::Odometry current_odom_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SMARTmBOTNode>());
  rclcpp::shutdown();
  return 0;
}