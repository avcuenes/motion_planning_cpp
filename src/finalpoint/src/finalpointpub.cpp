#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"


using namespace std::chrono_literals;


/**
 * @brief A class Target Point.
 *
 * This class publish information about target point.
 */

class TargetPoint : public rclcpp::Node
{
  public:
    TargetPoint()
    : Node("TargetPoint"), count_(0)
    { 

      finalpointpub_ = this->create_publisher<geometry_msgs::msg::Point>("finalpoint", 10);  /*!< İnitilaze topic */
      //! create timer for callback function
      timer_ = this->create_wall_timer(
      500ms, std::bind(&TargetPoint::timer_callback, this));
    }

  private:
    //! Timer callback function 
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Point();
      message.x = 1.0;  /*!< X position of final point */
      message.y = 1.0;  /*!< Y position of final point */
      message.z = 1.0;  /*!< Z position of final point */
      //! Info about published topic
      RCLCPP_INFO(get_logger(), "Published: [%.2f, %.2f, %.2f]",
                message.x, message.y, message.z);
      
      //! Publish message
      finalpointpub_->publish(message);
    }
    //! Create timer with ros2
    rclcpp::TimerBase::SharedPtr timer_;
    //! Create publisher 
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr finalpointpub_;
    size_t count_;
};


/**
 * @brief Main function to demonstrate the usage of the Target class.
 *
 * This program initilazie ros2 init and spin function
 *
 * @return 0 on success.
 */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetPoint>());
  rclcpp::shutdown();
  return 0;
}