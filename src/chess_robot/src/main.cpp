#include "rclcpp/rclcpp.hpp"
#include "chess_robot/robot_controller.h"

#include <thread>
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); 

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting program...");

    auto chess_node = std::make_shared<ChessRobotController>();

    std::thread spin_thread([chess_node]() { 
        rclcpp::spin(chess_node); 
    });

    // Choose mode: runDebug() or run_chess_game()
    chess_node->runDebug(); 

    rclcpp::shutdown(); 
    if (spin_thread.joinable()) spin_thread.join();

    return 0;
}