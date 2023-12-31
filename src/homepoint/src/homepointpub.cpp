#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"


using namespace std::chrono_literals;


/**
 * @brief A class Home Point.
 *
 * This class publish information about target point.
 */

class HomePoint : public rclcpp::Node
{
  public:
    HomePoint()
    : Node("HomePoint"), count_(0)
    { 

      homepointpub_ = this->create_publisher<geometry_msgs::msg::Point>("homepoint", 10);  /*!< İnitilaze topic */
      //! create timer for callback function
      timer_ = this->create_wall_timer(
      500ms, std::bind(&HomePoint::timer_callback, this));
    }

  private:
    //! Timer callback function 
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Point();
      message.x = 10.0;  /*!< X position of home point */
      message.y = 10.0;  /*!< Y position of home point */
      message.z = 10.0;  /*!< Z position of home point */
      //! Info about published topic
      RCLCPP_INFO(get_logger(), "Published: [%.2f, %.2f, %.2f]",
                message.x, message.y, message.z);
      
      //! Publish message
      homepointpub_->publish(message);
    }
    //! Create timer with ros2
    rclcpp::TimerBase::SharedPtr timer_;
    //! Create publisher 
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr homepointpub_;
    size_t count_;
};


/**
 * @brief Main function to demonstrate the usage of the Home class.
 *
 * This program initilazie ros2 init and spin function
 *
 * @return 0 on success.
 */

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomePoint>());
  rclcpp::shutdown();
  return 0;
}