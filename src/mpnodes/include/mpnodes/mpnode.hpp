#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"


using namespace std::chrono_literals;

/**
 * @brief Obstacle types
 * 
*/
enum obstacletype {
      square = 0,
      circle
    }

/**
 * @brief This struct define obstacles
*/
struct Obstacle {
    double x, y,r;
    obstacletype typeofobstacle;
    Obstacle(double x_, double y_, double r_) : x(x_), y(y_), r(r_) {}
};


/**
 * @brief A class Obstacle Point Point.
 *
 * This class publish information about target point.
 */

class MPNODE : public rclcpp::Node
{
  public:
    MPNODE();

  private:
    std::vector<Obstacle*>  Obtacles;
    //! Timer callback function 
    void timer_callback();
    //! Create timer with ros2
    rclcpp::TimerBase::SharedPtr timer_;
    // Define subscriptions types
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr obstaclepoints_; 
    size_t count_;
};


