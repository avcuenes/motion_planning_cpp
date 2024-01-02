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
    homepoint_ = this->create_subscription<geometry_msgs::msg::Point>(
      "topic", 10, std::bind(&MPNODE::homepoint_callback, this, _1));
    finalpoint_ = this->create_subscription<geometry_msgs::msg::Point>(
      "finalpoint", 10, std::bind(&MPNODE::finalpoint_callback, this, _1));
    
    
}

// finalpoint callback function
void MPNODE::homepoint_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{ 

  HomePoint.x = static_cast<double>(msg->x);
  HomePoint.y = static_cast<double>(msg->y);
  HomePoint.z = static_cast<double>(msg->z);
  
  RCLCPP_INFO(get_logger(), "obstacle point: [%.2f, %.2f, %.2f]",
    HomePoint.x, HomePoint.y,HomePoint.z );

}


// obstacle callback function
void MPNODE::finalpoint_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{ 

  FinalPoint.x = static_cast<double>(msg->x);
  FinalPoint.y = static_cast<double>(msg->y);
  FinalPoint.z = static_cast<double>(msg->z);
  
  RCLCPP_INFO(get_logger(), "obstacle point: [%.2f, %.2f, %.2f]",
    FinalPoint.x, FinalPoint.y,FinalPoint.z );
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