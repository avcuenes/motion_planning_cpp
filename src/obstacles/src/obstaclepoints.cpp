#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"


using namespace std::chrono_literals;


/**
 * @brief A class Obstacle Point Point.
 *
 * This class publish information about target point.
 */

class ObstaclePoint : public rclcpp::Node
{
  public:
    ObstaclePoint()
    : Node("ObstaclePoint"), count_(0)
    { 

      obstaclepointpub_ = this->create_publisher<geometry_msgs::msg::Point>("obstaclepoint", 10);  /*!< İnitilaze topic */
      //! create timer for callback function
      timer_ = this->create_wall_timer(
      500ms, std::bind(&ObstaclePoint::timer_callback, this));
    }

  private:
    //! Timer callback function 
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Point();
      message.x = 5.0;  /*!< X position of obstacle point */
      message.y = 5.0;  /*!< Y position of obstacle point */
      message.z = 1.0;  /*!< Radiıes  point */
      //! Info about published topic
      RCLCPP_INFO(get_logger(), "Published: [%.2f, %.2f, %.2f]",
                message.x, message.y, message.z);
      
      //! Publish message
      obstaclepointpub_->publish(message);
    }
    //! Create timer with ros2
    rclcpp::TimerBase::SharedPtr timer_;
    //! Create publisher 
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr obstaclepointpub_;
    size_t count_;
};


/**
 * @brief Main function to demonstrate the usage of the Obstacle class.
 *
 * This program initilazie ros2 init and spin function
 *
 * @return 0 on success.
 */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePoint>());
  rclcpp::shutdown();
  return 0;
}