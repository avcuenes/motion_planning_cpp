#include <cstdio>
#include "mpnodes/mpnode.hpp"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <iostream>
using std::placeholders::_1;



MPNODE::MPNODE()
: Node("MotionPlanning")
{ 

   // Define topics for subscribers
    obstaclepoints_ = this->create_subscription<geometry_msgs::msg::Point>(
      "topic", 10, std::bind(&MPNODE::obstacle_callback, this, _1));
}


// obstacle callback function
void MPNODE::obstacle_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{ 
  Obtacles.clear();

  double obstacleX = static_cast<double>(msg->x);
  double obstacleY = static_cast<double>(msg->y);
  double obstacleR = static_cast<double>(msg->z);
  
  RCLCPP_INFO(get_logger(), "obstacle point: [%.2f, %.2f, %.2f]",
    obstacleX, obstacleY,obstacleR );

  Obstacle* newobstacle = new Obstacle(obstacleX,obstacleY,obstacleR);
  newobstacle->typeofobstacle = obstacletype::circle;
  Obtacles.push_back(newobstacle);
  

}

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
  rclcpp::spin(std::make_shared<MPNODE>());
  rclcpp::shutdown();
  return 0;
}