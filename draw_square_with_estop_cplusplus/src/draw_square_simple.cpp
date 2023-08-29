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
  enum State
  {
    connecting_to_subscriber = 0,
    initializing = 1,
    driving_straight = 2,
    turning_left = 3,
    done = 4
  };
  public:
    State state = initializing;
    DrawSquare()
  : Node("draw_square_cplusplus"), e_stop(false)
  {
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer = rclcpp::create_timer(this,
                                 this->get_clock(),
                                 rclcpp::Duration::from_seconds(0.1),
                                 std::bind(&DrawSquare::run_loop, this));
    estop_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "estop",
      10,
      std::bind(
        &DrawSquare::handle_estop,
        this,
        std::placeholders::_1));
    state_entry_time = this->get_clock()->now();
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

  void enter_state(const State& newState) {
    switch (newState) {
    case initializing:
        break;
    case connecting_to_subscriber:
        drive(0.0, 0.0);
        break;
    case driving_straight:
        drive(forward_vel, 0.0);
        break;
    case turning_left:
        drive(0.0, angular_vel);
        break;
    case done:
        drive(0.0, 0.0);
        break;
    }
    state = newState;
    state_entry_time = this->get_clock()->now();
  }

  /// This is the main logic for piloting the square
  void run_loop() {
    if (e_stop) {
        return;
    }
    auto elapsed = this->get_clock()->now() - state_entry_time;
    switch (state) {
    case initializing:
        if (elapsed > rclcpp::Duration::from_seconds(1.0)) {
            enter_state(connecting_to_subscriber);
        }
        break;
    case connecting_to_subscriber:
        if (elapsed > rclcpp::Duration::from_seconds(1.0)) {
            enter_state(driving_straight);
        }
        break;
    case driving_straight:
        if (elapsed > rclcpp::Duration::from_seconds(side_length / forward_vel)) {
            enter_state(turning_left);
        }
        break;
    case turning_left:
        if (elapsed > rclcpp::Duration::from_seconds(M_PI / 2.0 / angular_vel)) {
            side_count++;
            if (side_count >= 4) {
                enter_state(done);
            } else {
                enter_state(driving_straight);
            }
        }
        break;
    case done:
        // sit here indefinitely
        break;
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
    static constexpr float angular_vel = 0.3f;
    static constexpr float forward_vel = 0.1f;
    static constexpr float side_length = 0.5f;
    int side_count = 0;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time state_entry_time;
    /// we declare e_stop atomic to enforce thread safety
    std::atomic<bool> e_stop;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DrawSquare>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}