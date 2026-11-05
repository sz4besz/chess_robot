
#include <cstddef>
#include <cstdlib>


#include <sstream>
#include <string>
#include <thread>
#include "game_state.h"
#include <geometry_msgs/msg/pose.hpp>
#include "chess_robot/robot_controller.h"
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>


using namespace std::chrono_literals;


ChessRobotController::ChessRobotController() :  Node("chess_robot_controller"), 
                                                engine("127.0.0.1", 9000), gs()
{
    RCLCPP_INFO(this->get_logger(), "Creating Robot Controller...");
    
    board_vision_client_ = this->create_client<std_srvs::srv::Trigger>("get_chess_fen");
    tile_vision_client_  = this->create_client<std_srvs::srv::Trigger>("get_tile");

    urscript_pub_ = this->create_publisher<std_msgs::msg::String>("/urscript_interface/script_command", 10);
}


void ChessRobotController::run_chess_game() 
{
    while(rclcpp::ok()) {
        
        std::string line = engine.readLine();
        // Step 1: wait for input
        if(line.empty()) continue;

        
        if(line.find("TRIGGER") != std::string::npos){

            // Step 2: request new board state
            std::string newBoard = request("board");
            RCLCPP_INFO(this->get_logger(), "FEN: %s", newBoard.c_str());

            // Step 3: detect move, make fen updates
            this->gs.detectMove(newBoard);
            std::string newFen = this->gs.buildFEN();

            // Step 4: ask engine for best move
            engine.sendCommand("position fen " + newBoard); 
            engine.sendCommand("go depth 15"); 
        }

        if(line.find("bestmove") != std::string::npos) {
            // Step 4: Engine responds 
            RCLCPP_INFO(this->get_logger(), "Engine move found: %s", line.c_str());
            
            if (line.length() < 13 || line.find("(none)") != std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "Game over or no move available.");
                continue; // Skip execution
            }

            // engine respond: bestmove e4e5 ponder e7e5, we need e4e5
            size_t space_idx = line.find(' ', 9);
            std::string bestmove;

            if (space_idx == std::string::npos) {
                // No space found after the move, read to the end of the line
                bestmove = line.substr(9); 
            } else {
                // Space found, slice exactly up to the space
                bestmove = line.substr(9, space_idx - 9); 
            } 
            
            // Step 5: asks for centers
            auto centers = request("centers");
            if (centers.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Vision frame dropped. Aborting move execution.");
                continue; 
}
            this->board_coordinates_ = pixelsToTCP(centers);
            
            // Step 6: parse the move type
            Tile fromTile = this->gs.AlgebraicToTile(bestmove.substr(0,2));
            Tile toTile = this->gs.AlgebraicToTile(bestmove.substr(2,2));
            MoveType type = this->gs.getMoveType(fromTile, toTile);

            // Step 6: make blacks move
            this->gs.move(fromTile, toTile);

            // Step 7: make the physical move
            executeMove(fromTile, toTile, type);
            
    }
           
    }
}

std::string ChessRobotController::request(std::string type) {

    auto client = (type == "board") ? board_vision_client_ : tile_vision_client_;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = client->async_send_request(request);

    if (result_future.wait_for(5s) == std::future_status::ready) {
        auto response = result_future.get();
        if (response->success) {
            return response->message; 
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "Vision request timed out or failed.");
    return "";
}

std::vector<geometry_msgs::msg::Point> ChessRobotController::pixelsToTCP(std::string centers){

    std::stringstream ss(centers);
    std::vector<geometry_msgs::msg::Point> tiles(64);
    std::string token;

    // Extract response from vision
    for(int i=0; i < 64; ++i){
        if (!(ss >> token)) break;
        size_t comma = token.find(',');
        tiles[i].x = std::stod(token.substr(0, comma));
        tiles[i].y = std::stod(token.substr(comma + 1));
        tiles[i].z = 0.0;
    }

    double scale_h, scale_v, mid_x, mid_y;
    ss >> scale_h >> scale_v >> mid_x >> mid_y;

    // TODO: MEASURE CAMERA OFFSETS IN METERS
    double cam_offset_x = 0.0;
    double cam_offset_y = 0.0;

    double dx = tiles[7].x - tiles[0].x;
    double dy = tiles[7].y - tiles[0].y;
    double theta = std::atan2(dy, dx);

    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    double midpoint_tcp_x = CAMERA_POSE.position.x + cam_offset_x;
    double midpoint_tcp_y = CAMERA_POSE.position.y + cam_offset_y;

    // Scales
    // TODO: MEASURE SQUARE SIZE
    double square_size = 0.025;
    double meters_per_pixel_h =  (7.0 * square_size) / scale_h;
    double meters_per_pixel_v =  (9.0 * square_size) / scale_v;

    std::vector<geometry_msgs::msg::Point> coords(64);

   for(int i=0; i < 64; ++i){

        double diff_x = (tiles[i].x - mid_x) * meters_per_pixel_h;
        double diff_y = (tiles[i].y - mid_y) * meters_per_pixel_v;
        double rotated_x = diff_x * cos_t - diff_y * sin_t;
        double rotated_y = diff_x * sin_t + diff_y * cos_t;

        // Map to robot coordinates
        coords[i].x = midpoint_tcp_x - rotated_y; 
        coords[i].y = midpoint_tcp_y - rotated_x; 
        coords[i].z = GRIP_HEIGHT;
}


    return coords;
}

// TODO
// Base orientation for entire program
const geometry_msgs::msg::Quaternion ChessRobotController::DOWN_ORIENTATION = [](){
    geometry_msgs::msg::Quaternion q;
    q.x = 3.14; 
    q.y = 0.0;
    q.z = 0.0;
    q.w = 0.0;
    return q;
}();

// TODO
// Home position for gripper
const geometry_msgs::msg::Pose ChessRobotController::HOME_POSE = [](){
    geometry_msgs::msg::Pose p;
    p.orientation = DOWN_ORIENTATION; 
    p.position.x = -0.09344;             
    p.position.y = -0.22674;             
    p.position.z = 0.06315;              
    return p;
}();

const geometry_msgs::msg::Pose ChessRobotController::CAMERA_POSE = [](){
    geometry_msgs::msg::Pose p;
    p.orientation = DOWN_ORIENTATION; 
    p.position.x = -0.09346;             
    p.position.y = -0.59265;             
    p.position.z = 0.42887;              
    return p;
}();



// TODO
// Graveyard for pieces someplace off board
const geometry_msgs::msg::Pose ChessRobotController::GRAVEYARD = [](){
    geometry_msgs::msg::Pose p;
    p.orientation = DOWN_ORIENTATION; 
    p.position.x = 0.30;             
    p.position.y = 0.00;             
    p.position.z = 0.40;              
    return p;
}();
// TODO
// The Z-plane where gripper contacts
const double ChessRobotController::GRIP_HEIGHT = -0.1818;

void ChessRobotController::normalMove(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to){

    // PICKUP
    // TODO: OPEN GRIPPER
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    movel_normal_script(from, to);
}

void ChessRobotController::executeMove(Tile fromTile, Tile toTile, MoveType type){

    int from_idx = static_cast<int>(fromTile);
    int to_idx = static_cast<int>(toTile);

    geometry_msgs::msg::Point from_pt = board_coordinates_[from_idx];
    geometry_msgs::msg::Point to_pt   = board_coordinates_[to_idx];
    
    switch(type){
        case MoveType::NORMAL: {
            normalMove(from_pt,to_pt);
            break;
        }
        case MoveType::CAPTURE: {
            normalMove(from_pt,to_pt);
            break;
        }
        case MoveType::CASTLE: {
            normalMove(from_pt,to_pt);
            break;
        }
        case MoveType::PROMOTION: {
            normalMove(from_pt,to_pt);
            break;
        }
        case MoveType::EN_PASSANT_CAPTURE: {
            normalMove(from_pt,to_pt);
            break;
        }
    }

}

//GRIPPER OPEN = 30mm
//GRIPPER CLOSE = 10mm
int OPEN = 40;
int CLOSE = 5;
int FORCE = 20;
void ChessRobotController::set_gripper(int width, int force){
    std::string cmd = "python3 /home/mates/onlab/src/chess_robot/scripts/robot_script/gripper.py " 
                      "--width " + std::to_string(width) + " --force " + std::to_string(force);
    std::system(cmd.c_str());

}

void ChessRobotController::movel(geometry_msgs::msg::Point pt, float height, std::string orientation) {

    std::string x = std::to_string(pt.x);
    std::string y = std::to_string(pt.y);
    std::string z = std::to_string(height);

    std::string script = "def move():\n";
    
    script += "movel(p[" + x + ", " + y + ", " + z + orientation + " a = 1.2, v=0.1, r=0)\n"; 
    script += "end\n";

    std_msgs::msg::String msg;
    msg.data = script;
    urscript_pub_->publish(msg);

}

void ChessRobotController::movel_normal_script(geometry_msgs::msg::Point from_pt, geometry_msgs::msg::Point to_pt){
/*
E.G. From documentation

"def my_prog():

  set_digital_out(1, True)

  movel(p[0.2, 0.3, 0.8, 0, 0, 3.14], a=1.2, v=0.25, r=0)

  textmsg(\"motion finished\")

  end"
  */
    std::string down = ", 0, 3.14, 0],";

    geometry_msgs::msg::Point home;
    home.x = HOME_POSE.position.x;
    home.y = HOME_POSE.position.y;
    home.z = HOME_POSE.position.z;

    int WAIT = 3;

    // Hover above pickup
    movel(from_pt, home.z, down); 
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));
    
    // OPEN GRIPPER (30mm, 20N)
    set_gripper(OPEN, FORCE);
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));

    // Descend to piece
    movel(from_pt, GRIP_HEIGHT, down);          
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));
    
    // CLOSE GRIPPER (10mm, 20N)
    set_gripper(CLOSE, FORCE);
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));

    // Ascend with piece
    movel(from_pt, home.z, down); 
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));

    // Hover above target
    movel(to_pt, home.z, down);   
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));

    // Descend to drop
    movel(to_pt, GRIP_HEIGHT + 0.15, down);       
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));

    // OPEN GRIPPER to drop piece
    set_gripper(OPEN, FORCE);
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));

    // Ascend from target
    movel(to_pt, home.z, down); 
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));

    // Return home
    movel(home, home.z, down);
    std::this_thread::sleep_for(std::chrono::seconds(WAIT));
}

void ChessRobotController::movej(geometry_msgs::msg::Point pt, float height, std::string orientation) {
    std::string x = std::to_string(pt.x);
    std::string y = std::to_string(pt.y);
    std::string z = std::to_string(height);

    std::string script = "def move():\n";
    // NOTE: Explicit comma added before 'a'
    script += "  movej(p[" + x + ", " + y + ", " + z + orientation + ", a=1.0, v=0.2, r=0)\n"; 
    script += "end\n";

    std_msgs::msg::String msg;
    msg.data = script;
    urscript_pub_->publish(msg);
}

void ChessRobotController::movej_camera_pos() {
    std::string camera_rot = ", 0.0, -2.201, 2.324]";
    std::string down_rot = ", 0.0, -3.1415, 0.0]";

    geometry_msgs::msg::Point home;
    home.x = HOME_POSE.position.x;
    home.y = HOME_POSE.position.y;
    home.z = HOME_POSE.position.z;

    geometry_msgs::msg::Point camera;
    camera.x = CAMERA_POSE.position.x;
    camera.y = CAMERA_POSE.position.y;
    camera.z = CAMERA_POSE.position.z;

    movel(home, home.z, down_rot);
    std::this_thread::sleep_for(std::chrono::seconds(5));
  
    movej(camera, camera.z, camera_rot);
    std::this_thread::sleep_for(std::chrono::seconds(5));
}

void ChessRobotController::movej_home_pos() {

    std::string down_rot = ", 0.0, -3.1415, 0.0]";

    geometry_msgs::msg::Point home;
    home.x = HOME_POSE.position.x;
    home.y = HOME_POSE.position.y;
    home.z = HOME_POSE.position.z;

    movej(home, home.z, down_rot);

    std::this_thread::sleep_for(std::chrono::seconds(5));
}


void ChessRobotController::runDebug(){
    RCLCPP_INFO(this->get_logger(), "Starting debug movement test...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    geometry_msgs::msg::Point home;
    home.x = HOME_POSE.position.x;
    home.y = HOME_POSE.position.y;
    home.z = HOME_POSE.position.z;

    geometry_msgs::msg::Point from;
    from.x = -0.07065;
    from.y = -0.30793;

    geometry_msgs::msg::Point to;
    to.x = 0.05844;
    to.y = -0.30794;

    //movej_camera_pos();
    //std::this_thread::sleep_for(std::chrono::seconds(5));

    //movej_home_pos();
    //std::this_thread::sleep_for(std::chrono::seconds(5));

    movel_normal_script(from, to);

    //movel(home, home.z, ", 0, 3.14, 0],");
    

    RCLCPP_INFO(this->get_logger(), "URScript command sent directly to the robot.");

}
