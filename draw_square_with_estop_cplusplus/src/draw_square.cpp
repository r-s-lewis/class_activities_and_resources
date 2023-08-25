/*
 * A reimplementation of the draw_square with e stop Python code
 */

#include <cstdio>

#include <atomic>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <math.h>

/// The Node that encapsulates our main functionality 
class DrawSquare : public rclcpp::Node {
  public:
    DrawSquare()
  : Node("draw_square_cplusplus"), e_stop(false)
  {
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    estop_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "estop",
      10,
      std::bind(
        &DrawSquare::handle_estop,
        this,
        std::placeholders::_1));
  }

  /**
   * The callback function to handle incoming messages on the estop topic 
   * 
   * @param msg the message that contains the possible e stop directive
   */
  void handle_estop(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      e_stop = true;
      drive(0.0, 0.0);
    }
  }

  /// This is the main logic for piloting the square
  void run_loop() {
    // TODO: properly support sim time
    // pause to make sure publisher and subscribers are ready
    std::this_thread::sleep_for(std::chrono::seconds(2));
    for (int i = 0; i < 4; i++) {
      if (!e_stop) {
        std::cout << "turning left" << std::endl;
        turn_left();
      }
      if (!e_stop) {
        std::cout << "driving forward" << std::endl;
        drive_forward(1.0);
      }
    }
    std::cout << "done with run loop" << std::endl;
  }

  /// Turn left 90 degrees using a time-based strategy
  void turn_left() {
    float angular_vel = 0.3;
    if (!e_stop) {
      drive(0.0, angular_vel);
      std::this_thread::sleep_for(std::chrono::duration<float>(M_PI / angular_vel));
      drive(0.0, 0.0);
    }
  }

  /**
   * Drive forward the specified (positive) distance
   * 
   * @param distance the distance in meters to drive (must be positive)
   */
  void drive_forward(float distance) {
    float forward_vel = 0.1;
    if (!e_stop) {
      drive(forward_vel, 0.0);
      std::this_thread::sleep_for(std::chrono::duration<float>(distance / forward_vel));
      drive(0.0, 0.0);
    }
  }

  /**
   * Set the robot's linear and angular velocity 
   * 
   * @param forward the linear velocity in meters per second 
   * @param angular the angular velocity in radians per second
   */
  void drive(float forward, float angular)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = forward;
    msg.angular.z = angular;
    publisher->publish(msg);
  }

  private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    /// we declare e_stop atomic to enforce thread safety
    std::atomic<bool> e_stop;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrawSquare>();
  std::shared_future<void> script = std::async(&DrawSquare::run_loop, node);
  // spin_until_future_complete allows us to handle normal callbacks while also executing
  // an additional function on a separate thread
  rclcpp::spin_until_future_complete(node, script);
  rclcpp::shutdown();
  return 0;
}