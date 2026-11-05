#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "chess_robot/engine_client.h"
#include "chess_robot/game_state.h"

class ChessRobotController : public rclcpp::Node {
private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urscript_pub_;

    EngineClient engine;
    GameState gs;

    static const geometry_msgs::msg::Point HOME_POS;
    static const geometry_msgs::msg::Point CAMERA_POS;
    static const geometry_msgs::msg::Point GRAVEYARD_POS;
    static const double GRIP_HEIGHT;

    std::vector<geometry_msgs::msg::Point> board_coordinates_;

public:
    ChessRobotController();
    void runDebug();
    void run_chess_game();
    
    std::string request(std::string type);
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr board_vision_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr tile_vision_client_;
    
    std::vector<geometry_msgs::msg::Point> pixelsToTCP(std::string centers);
    void executeMove(Tile fromTile, Tile toTile, MoveType type);

    //TODO: REMOVE
    void normalMove(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to);

    void movel_normal_script(geometry_msgs::msg::Point from_pt, geometry_msgs::msg::Point to_pt);
    void movel(geometry_msgs::msg::Point pt, float height, std::string orientation);

    void movej(geometry_msgs::msg::Point pt, float height, std::string orientation);
    void movej_camera_pos();
    void movej_home_pos();

    void set_gripper(int width, int force);

};

#endif // ROBOT_CONTROLLER_HPP