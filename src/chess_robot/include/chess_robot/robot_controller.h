#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <memory>
#include <string>
#include <vector>


#include <moveit/move_group_interface/move_group_interface.h>


// Required message definition types for your method parameters and constants
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "chess_robot/engine_client.h"
#include "chess_robot/game_state.h"

class ChessRobotController : public rclcpp::Node {
private:

    std::vector<double> start_position_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr vision_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urscript_pub_;

    EngineClient engine;
    GameState gs;

    static const geometry_msgs::msg::Quaternion DOWN_ORIENTATION;
    static const geometry_msgs::msg::Pose HOME_POSE;
    static const geometry_msgs::msg::Pose CAMERA_POSE;
    static const geometry_msgs::msg::Pose GRAVEYARD;
    static const double GRIP_HEIGHT;

    std::vector<geometry_msgs::msg::Point> board_coordinates_;

public:
    ChessRobotController();
    auto getMoveGroup();
    bool move_pose(std::vector<double> position);
    void initialize_move_group();
    void runDebug();
    void run_chess_game();
    
    bool move_linear(const std::vector<geometry_msgs::msg::Pose>& waypoints);
    
    void print_current_tcp();
    bool move_to_pose(geometry_msgs::msg::Pose target_pose);
    std::string request(std::string type);
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr board_vision_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr tile_vision_client_;
    
    std::vector<geometry_msgs::msg::Point> pixelsToTCP(std::string centers);
    void executeMove(Tile fromTile, Tile toTile, MoveType type);

    geometry_msgs::msg::Pose createPose(double x, double y, double z);
    void normalMove(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to);

    void movel_normal_script(geometry_msgs::msg::Point from_pt, geometry_msgs::msg::Point to_pt);
    void movel(geometry_msgs::msg::Point pt, float height, std::string orientation);

    void movej(geometry_msgs::msg::Point pt, float height, std::string orientation);
    void movej_camera_pos();
    void movej_home_pos();

    void set_gripper(int width, int force);
};

#endif // ROBOT_CONTROLLER_HPP