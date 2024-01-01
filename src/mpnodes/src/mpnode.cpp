#include <cstdio>
#include "mpnodes/mpnode.hpp"

MPNODE::MPNODE()
: Node("ObstaclePoint"), count_(0)
{ 

   // Define topics for subscribers
    obstaclepoints_ = this->create_subscription<geometry_msgs::msg::Point>(
      "obstacle_point", 10, std::bind(&MP::obstacle_callback, this, _1));
}


// obstacle callback function
void MPNODE::obstacle_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{ 
  
  for (size_t i = 0; i < msg->data.size(); i += 3) {

      double obstacleX = static_cast<double>(msg->x);
      double obstacleY = static_cast<double>(msg->y);
      double obstacleR = static_cast<double>(msg->z);
      
      RCLCPP_INFO(get_logger(), "obstacle point: [%.2f, %.2f, %.2f]",
        obstacleX, obstacleY,obstacleR );

      Obstacle* newobstacle = new Obstacle(obstacleX,obstacleY,obstacleR);
      Obtacles.push_back(newlocale);
  }

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