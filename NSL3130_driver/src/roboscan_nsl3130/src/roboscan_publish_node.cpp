#include "rclcpp/rclcpp.hpp"
#include <signal.h>
#include <cstdlib>

#include "nsl3130_driver.h"


//===================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Nsl3130Driver>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    printf("ROS Finish\n");

}








