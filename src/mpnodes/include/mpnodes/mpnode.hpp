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
    };

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
    //! Obstacle callback function 
    void obstacle_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    //! Homepoint callback function
    void homepoint_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    //! Fİnalpoint callback function
    void finalpoint_callback(const geometry_msgs::msg::Point::SharedPtr msg);

    /**
     * @brief This struct define homepoint location
    */
    struct {
        double x, y,z;
    }HomePoint,FinalPoint;


    //! Create timer with ros2
    rclcpp::TimerBase::SharedPtr timer_;
    // Define subscriptions types
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr obstaclepoints_; 
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr homepoint_; 
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr finalpoint_; 
    
    size_t count_;
};


