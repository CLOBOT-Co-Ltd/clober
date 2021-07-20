
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "clober_serial/clober_serial.hpp"

int main(int argc, char** argv){
    std::cout <<"clober_serial"<<std::endl;

    rclcpp::init(argc, argv);
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::SingleThreadedExecutor executor;
    
    auto clober_serial = std::make_shared<CloberSerial>();
    executor.add_node(clober_serial);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}