#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/simple_fsm.hpp"
#include "ds4_driver_msgs/msg/status.hpp"
#include <cmath>

#include "std_msgs/msg/int16.hpp"
#include "ros2_interface/msg/robot.hpp"
#include "ros2_interface/msg/robot_array.hpp"
#include "ros2_interface/msg/terminal.hpp"
#include "ros2_interface/msg/terminal_array.hpp"
#include "ros2_interface/srv/robot_req.hpp"
#include "ros2_interface/srv/terminal_req.hpp"

#include "communication/nlohmann_json.hpp"
#include "cpr/cpr.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <boost/algorithm/string.hpp>

#include "math.h"
#include "stdint.h"
#include "stdio.h"
#include "sys/ioctl.h"
#include "termios.h"
#include <sys/time.h>
#include <unistd.h>
#include <string>
#include <stdarg.h>

#define ENDPOINT_GET_POSE "/reeman/pose"
#define ENDPOINT_GET_MODE "/reeman/get_mode"
#define ENDPOINT_GET_PWR_MGMT "/reeman/base_encode"
#define ENDPOINT_OFFSET_POSE "/cmd/reloc_absolute"
#define ENDPOINT_GO2TARGET "/cmd/nav"
#define ENDPOINT_CANCEL_GO2TARGET "/cmd/cancel_goal"
#define ENDPOINT_GO2CHARGE "/cmd/charge"
#define ENDPOINT_SET_MAX_TRANS_SPEED "/cmd/max_speed"
#define ENDPOINT_SET_MAX_ROT_SPEED "/reeman/navi_routes"
#define ENDPOINT_SET_JACKUP "/cmd/hydraulic_up"
#define ENDPOINT_SET_JACKDOWN "/cmd/hydraulic_down"
#define ENDPOINT_GET_NAV_STATUS "/reeman/nav_status"
#define ENDPOINT_GET_ROUTE_WAYPOINTS "/reeman/position"

#define ENDPOINT_GET_SPEED "/reeman/speed"
#define ENDPOINT_POST_CMD_SPEED "/cmd/speed"
#define ENDPOINT_SET_MODE "/cmd/set_mode"
#define ENDPOINT_SAVE_MAP "/cmd/save_map"
#define ENDPOINT_GET_MAP_LISTS "/reeman/history_map"
#define ENDPOINT_GET_CURRENT_MAP_NAME "/reeman/current_map"

#define FSM_INIT 0
#define FSM_SAFEOP 1
#define FSM_OPERATION 2
#define FSM_DOING_CHARGING 3
#define FSM_TEST_MULTIROBOT 4

typedef struct
{
    float target_pose_x;
    float target_pose_y;
    float target_pose_yaw;
} route_t;

typedef struct {
    std::string name;
    std::string type;
    float x;
    float y;
    float theta;
} route_waypoints_t;

// Routing for Robot Movement
typedef struct {
    uint8_t terminal_type;

    int16_t terminal_id;

    float target_pose_x;
    float target_pose_y;
    float target_pose_theta;

    float target_max_velocity_x;
    float target_max_velocity_y;
    float target_max_velocity_theta;

    float toleransi_posisi;
    float toleransi_sudut;

    std::string locked_by_robot_ip;

    int16_t danger_value;
} terminal_t;

typedef struct {
    std::string ip_addr;

    float init_pose_x;
    float init_pose_y;
    float init_pose_theta;
    
    float target_pose_x;
    float target_pose_y;
    float target_pose_theta;

    float max_velocity_x;
    float max_velocity_y;
    float max_velocity_theta;

    float position_error_tolerance;
    float angle_error_tolerance;

    float distance_to_goal;

    uint8_t current_mode;
    float battery_soc;
    uint8_t emergency_button_enabled;
    uint16_t nav_status[2];

    int16_t current_terminal;
    int16_t next_terminal;

    std::vector<int16_t> routes;
    std::vector<int16_t> routes_init;

    int16_t current_route_index;
} robot_config_t;

ros2_interface::msg::RobotArray all_robot;
ros2_interface::msg::TerminalArray all_terminal;

class IOReeman : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    HelpLogger logger;
    MachineState fsm_robot;

    // Configs
    std::string reeman_controller_ip = "10.7.101.213";
    // std::string reeman_controller_ip = "";

    int threshold_minimum_request_period_ms = 1000; // Minimum period between requests in milliseconds
    float threshold_minimum_battery_soc = 20;

    // Robot vars
    float final_pose_x = 0.0f;
    float final_pose_y = 0.0f;
    float final_pose_yaw = 0.0f;
    uint8_t emergency_button = 0;
    float battery_soc = 0.0f;
    std::vector<route_t> route_list;
    uint8_t current_route_index = 0;
    
    // Sesuatu
    uint64_t global_counter_ms = 0;
    uint64_t last_time_request_counter_ms = 0;
    bool request_to_start = false;
    bool request_to_stop = false;

    // KBHIT
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_keyboard_hit;
    int16_t key_val;

    // ds4 vars
    rclcpp::Subscription<ds4_driver_msgs::msg::Status>::SharedPtr sub_ds4;
    ds4_driver_msgs::msg::Status ds4_current_status;
    ds4_driver_msgs::msg::Status ds4_previous_status;
    float max_vx = 0.0f;
    float max_vth = 0.0f;
    uint16_t counter_stop_command = 0;

    // Waypoints route vars
    std::vector<route_waypoints_t> route_waypoints_list;
    uint8_t robot_current_mode;
    uint16_t robot_operation_status = 0;

    // Mapping mode
    uint8_t set_mapping_mode = 1;

    // Routing
    std::vector<terminal_t> current_terminals;
    robot_config_t current_robot_config;
    std::string configs_path = "";
    std::string terminals_path = "";
    std::string robot_config_path = "";

    // Service
    rclcpp::Service<ros2_interface::srv::RobotReq>::SharedPtr srv_robot_req;
    rclcpp::Service<ros2_interface::srv::TerminalReq>::SharedPtr srv_terminal_req;
    
    rclcpp::Publisher<ros2_interface::msg::RobotArray>::SharedPtr pub_robot_array;
    rclcpp::Subscription<ros2_interface::msg::TerminalArray>::SharedPtr sub_terminal_update;

    uint8_t bs_reload_process = 1;

    IOReeman()
        : Node("io_reeman")
    {
        this->declare_parameter("reeman_controller_ip", reeman_controller_ip);
        this->get_parameter("reeman_controller_ip", reeman_controller_ip);

        this->declare_parameter("threshold_minimum_request_period_ms", threshold_minimum_request_period_ms);
        this->get_parameter("threshold_minimum_request_period_ms", threshold_minimum_request_period_ms);

        this->declare_parameter("threshold_minimum_battery_soc", threshold_minimum_battery_soc);
        this->get_parameter("threshold_minimum_battery_soc", threshold_minimum_battery_soc);
        
        this->declare_parameter("max_vx", max_vx);
        this->get_parameter("max_vx", max_vx);

        this->declare_parameter("max_vth", max_vth);
        this->get_parameter("max_vth", max_vth);

        this->declare_parameter("configs_path", configs_path);
        this->get_parameter("configs_path", configs_path);

        terminals_path = configs_path + "terminals.csv";
        robot_config_path = configs_path + "robot_config.csv";

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        std::cout << configs_path << std::endl;
        std::cout << terminals_path << std::endl;
        std::cout << robot_config_path << std::endl;
        load_terminals();
        load_robot_config();

        for (const auto& terminal : all_terminal.terminals) {
            logger.info("Terminal ID: %d, Type: %d, Target Pose X: %.2f, Y: %.2f, Theta: %.2f",
                terminal.terminal_id, terminal.terminal_type,
                terminal.target_pose_x, terminal.target_pose_y, terminal.target_pose_theta);
        }

        for (const auto& terminal : current_terminals) {
            logger.info("current_robot_terminals: target_pose_x: %.2f target_pose_y: %.2f target_pose_theta: .%2f", terminal.target_pose_x, terminal.target_pose_y, terminal.target_pose_theta);
        }

        for (const auto& route : current_robot_config.routes) {
            logger.info("robot_routes: %d", route);
        }

        fsm_robot.value = FSM_SAFEOP;
        // fsm_robot.value = FSM_INIT;

        sleep(2);
        sub_ds4 = this->create_subscription<ds4_driver_msgs::msg::Status>("/ds4/status", 1, std::bind(&IOReeman::callback_ds4, this, std::placeholders::_1));
        sub_keyboard_hit = this->create_subscription<std_msgs::msg::Int16>("/key_pressed", 1, std::bind(&IOReeman::callback_kbhit, this, std::placeholders::_1));

        sub_terminal_update = this->create_subscription<ros2_interface::msg::TerminalArray>("/basestation/terminal_update", 1, std::bind(&IOReeman::callback_terminal_update, this, std::placeholders::_1));

        pub_robot_array = this->create_publisher<ros2_interface::msg::RobotArray>("/communication/robot_array", 1);

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&IOReeman::callback_routine, this));

        srv_robot_req = this->create_service<ros2_interface::srv::RobotReq>("/srv/communication/robot_req", std::bind(&IOReeman::callback_srv_robot_req, this, std::placeholders::_1, std::placeholders::_2));
        srv_terminal_req = this->create_service<ros2_interface::srv::TerminalReq>("/srv/communication/terminal_req", std::bind(&IOReeman::callback_srv_terminal_req, this, std::placeholders::_1, std::placeholders::_2));

        logger.info("IOReeman node initialized");
    }

    ~IOReeman()
    {
    }

    void callback_kbhit(std_msgs::msg::Int16::SharedPtr msg) {
        switch (msg->data) {
            case 'w':
                key_val = 'w';
                break;
            case 's':
                key_val = 's';
                break;
            case 'a':
                key_val = 'a';
                break;
            case 'd':
                key_val = 'd';
                break;
            case ' ':
                key_val = ' ';
                break;
            case '1':
                key_val = '1';
                break;
            case '9':
                key_val = '9';
                break;
            case '2':
                key_val = '2';
                break;
        }

        logger.info("Key pressed: %c", key_val);
    }

    void keyboard_handler() {
        switch(key_val) {
            case 'w':
                send_speed_command(0.4, 0);
                break;
            case 's':
                send_speed_command(-0.4, 0);
                break;
            case 'a':
                send_speed_command(0, 0.3);
                break;
            case 'd':
                send_speed_command(0, -0.3);
                break;
            case ' ':
                send_stop_command();
                break;
            case '1':
                robot_operation_status = 1;
                break;
            case '9':
                robot_operation_status = 9;
                break;
            case '2':
                robot_operation_status = 2;
                break;
            case '3':
                robot_operation_status = 3;
                break;
        }
    }
    // ===================================================================================================================
    
    void callback_srv_terminal_req(const ros2_interface::srv::TerminalReq::Request::SharedPtr request, ros2_interface::srv::TerminalReq::Response::SharedPtr response) {
        response->terminals = all_terminal.terminals;
    }

    void callback_srv_robot_req(const ros2_interface::srv::RobotReq::Request::SharedPtr request, ros2_interface::srv::RobotReq::Response::SharedPtr response) {
        response->robots = all_robot.robots;
    }

    void callback_terminal_update(const ros2_interface::msg::TerminalArray::SharedPtr msg) {
        ros2_interface::msg::TerminalArray updated_terminals = all_terminal;
        
        for (auto& current_terminal: updated_terminals.terminals) {
            for (const auto& received_terminal: msg->terminals) {
                if (current_terminal.terminal_id == received_terminal.terminal_id) {
                    current_terminal.target_pose_x = received_terminal.target_pose_x;
                    current_terminal.target_pose_y = received_terminal.target_pose_y;
                    current_terminal.target_pose_theta = received_terminal.target_pose_theta;
                    current_terminal.target_max_velocity_x = received_terminal.target_max_velocity_x;
                    current_terminal.target_max_velocity_y = received_terminal.target_max_velocity_y;
                    current_terminal.target_max_velocity_theta = received_terminal.target_max_velocity_theta;
                    current_terminal.toleransi_posisi = received_terminal.toleransi_posisi;
                    current_terminal.toleransi_sudut = received_terminal.toleransi_sudut;
                    current_terminal.locked_by_robot_ip = received_terminal.locked_by_robot_ip;
                    current_terminal.danger_value = received_terminal.danger_value;
                } else {
                    updated_terminals.terminals.push_back(received_terminal);
                }
            }
        }

        save_terminals(updated_terminals);
    }

    void callback_ds4(const ds4_driver_msgs::msg::Status::SharedPtr msg) {
        ds4_current_status = *msg;
        // logger.info("axis_left_y: %.2f || axis_right_x: %.2f", ds4_current_status.axis_left_y, ds4_current_status.axis_right_x);
    }

    bool do_request()
    {
        if (global_counter_ms - last_time_request_counter_ms < threshold_minimum_request_period_ms)
        {
            logger.warn("Skipping request, not enough time has passed since the last request");
            return false;
        }
    }

    std::string build_url(const std::string &endpoint, const std::string &ip_addr = "")
    {
        std::string actual_ip = ip_addr.empty() ? reeman_controller_ip : ip_addr;
        if (actual_ip.empty())
        {
            logger.error("Reeman controller IP is not set");
            return "";
        }
        return "http://" + actual_ip + endpoint;
    }

    void get_route_lists()
    {   
        cpr::Response res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_MODE)));
        nlohmann::json json_res = nlohmann::json::parse(res.text);

        for (const auto &item : json_res["waypoints"]) {
            route_waypoints_t waypoint_buffer;
            waypoint_buffer.name = item["name"];
            waypoint_buffer.type = item["type"];
            waypoint_buffer.x = item["pose"]["x"];
            waypoint_buffer.y = item["pose"]["y"];
            waypoint_buffer.theta = item["pose"]["theta"];
            route_waypoints_list.push_back(waypoint_buffer);

            // printf("Name: %s, Type: %s, || Pose x=%.2f, y=%.2f, theta=%.2f\n",
                //    waypoint_buffer.name.c_str(), waypoint_buffer.type.c_str(),
                //    waypoint_buffer.x, waypoint_buffer.y, waypoint_buffer.theta);
        }
    }

    void transmit_all() {
        for (auto& robot : all_robot.robots) {
            cpr::Response res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_POSE, robot.ip_addr)));
            nlohmann::json json_res = nlohmann::json::parse(res.text);
            
            robot.current_pose_x = json_res["x"];
            robot.current_pose_y = json_res["y"];
            robot.current_pose_theta = json_res["theta"];

            res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_PWR_MGMT, robot.ip_addr)));
            json_res = nlohmann::json::parse(res.text);
            robot.battery_soc = json_res["battery"];
            robot.emergency_button = json_res["emergencyButton"];

            res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_MODE, robot.ip_addr)));
            json_res = nlohmann::json::parse(res.text);
            robot.mode = json_res["mode"];

            res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_NAV_STATUS, robot.ip_addr)));
            json_res = nlohmann::json::parse(res.text);
            robot.nav_state = json_res["res"];
            robot.nav_code = json_res["reason"];
            robot.dist_to_goal = json_res["dist"];
            robot.mileage = json_res["mileage"];
            
            res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_SPEED, robot.ip_addr)));
            json_res = nlohmann::json::parse(res.text);
            robot.current_velocity_x = json_res["vx"];
            robot.current_velocity_theta = json_res["vth"];
        }

        pub_robot_array->publish(all_robot);
    }

    // ===================================================================================================================

    void get_motion_nav_status(uint16_t *nav_status, float *error_position_m)
    {
        cpr::Response res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_NAV_STATUS)));
        nlohmann::json json_res = nlohmann::json::parse(res.text);
        nav_status[0] = json_res["res"];
        nav_status[1] = json_res["reason"];
        *error_position_m = json_res["dist"];

        printf("dist_to_goal: %.2f, state: %d, code: %d\n", *error_position_m, nav_status[0], nav_status[1]);
    }

    bool motion_to_point(float target_x, float target_y, float target_yaw, float max_speed = 0.3, float position_error_tolerance = 0.5, float angle_error_tolerance = 0.5)
    {
        if (robot_current_mode != 2) {
            logger.error("Robot is not in navigation mode, cannot move to point");
            return false;
        }

        static uint8_t motion_to_point_status = 0;

        if (motion_to_point_status == 0) {
            cpr::Response res = cpr::Post(cpr::Url(build_url(ENDPOINT_SET_MAX_TRANS_SPEED)),
                                cpr::Body("{\"speed\":" + std::to_string(max_speed) + "}"),
                                cpr::Header{{"Content-Type", "application/json"}});

            sleep(0.100);

            res = cpr::Post(cpr::Url(build_url(ENDPOINT_GO2TARGET)),
                                      cpr::Body("{\"x\":" + std::to_string(target_x) + ",\"y\":" + std::to_string(target_y) + ",\"theta\":" + std::to_string(target_yaw) + "}"),
                                      cpr::Header{{"Content-Type", "application/json"}});

            motion_to_point_status = 1;
        } else if (motion_to_point_status == 1) {
            float target_error_position = sqrtf(powf(final_pose_x - target_x, 2) + powf(final_pose_y - target_y, 2));
            float target_error_angular = fabsf(final_pose_yaw - target_yaw);

            if (target_error_position < position_error_tolerance && target_error_angular < angle_error_tolerance) {
                motion_to_point_status = 0;
                return true; 
            } 

            printf("Moving to x: %.2f, y: %.2f, yaw: %.2f\n", target_x, target_y, target_yaw);
            printf("Still moving to point: target error = %.2f\n", target_error_position);
        }
        
        return false;
    }

    bool motion_cancel()
    {
        cpr::Response res = cpr::Post(cpr::Url(build_url(ENDPOINT_CANCEL_GO2TARGET)),
                                      cpr::Body("{}"),
                                      cpr::Header{{"Content-Type", "application/json"}});
        
        return res.status_code == 200;
    }

    void send_speed_command(float vx, float vth){
        char v_body[128];
        snprintf(v_body, sizeof(v_body), "{\"vx\":%.2f,\"vth\":%.2f}", vx, vth);
        // logger.info("Command JSON: %s", v_body);

        cpr::Response res_post = cpr::Post(
            cpr::Url(build_url(ENDPOINT_POST_CMD_SPEED)),
            cpr::Body(std::string(v_body)),
            cpr::Header{{"Content-Type", "application/json"}}
        );

        if (res_post.error)
        {
            logger.error("Failed to send speed command: %s", res_post.error.message.c_str());
            return;
        }
    }

    void send_stop_command()
    {
        if(counter_stop_command++ >= 100){
            counter_stop_command = 100;
            return;
        }

        send_speed_command(0.0f, 0.0f);
    }

    // ===================================================================================================================

    void get_robot_global_data() {
        cpr::Response res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_POSE)));
        nlohmann::json json_res = nlohmann::json::parse(res.text);
    
        final_pose_x = json_res["x"];
        final_pose_y = json_res["y"];
        final_pose_yaw = json_res["theta"];
            
        res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_PWR_MGMT)));
        json_res = nlohmann::json::parse(res.text);
        battery_soc = json_res["battery"];
        emergency_button = json_res["emergencyButton"];

        res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_MODE)));
        json_res = nlohmann::json::parse(res.text);
        robot_current_mode = json_res["mode"];

        printf("Robot current mode: %d\n", robot_current_mode);
        printf("Final Pose: x=%.2f, y=%.2f, yaw=%.2f\n", final_pose_x, final_pose_y, final_pose_yaw);
        printf("Battery SOC: %.2f%%, Emergency Button: %d\n", battery_soc, emergency_button);

        get_motion_nav_status(current_robot_config.nav_status, &current_robot_config.distance_to_goal);
    }
    
    void callback_routine()
    {
        switch (fsm_robot.value)
        {
        case FSM_INIT:
        {
            break;
        }
        
        case FSM_SAFEOP:
        {
            get_robot_global_data();
            if (request_to_start)
            {
                if (route_list.size() > 0)
                {
                    fsm_robot.value = FSM_OPERATION;
                    break;
                }
            }

            // if (battery_soc < threshold_minimum_battery_soc)
            // {
            //     logger.warn("Battery SOC is below threshold");
            // }

            // logger.info("%.2f %.2f", ds4_current_status.axis_left_y, ds4_current_status.axis_right_x);
            // printf("BUTTON CURRENT X: %d\n", ds4_current_status.button_cross);
            // printf("BUTTON PREVIOUS X: %d\n", ds4_previous_status.button_cross);
            // printf("============================================\n");

            // ds4_handler();
            keyboard_handler();
            // get_route_lists();
            operation_manager();
            break;
        }

        case FSM_OPERATION:
        {
            // Jika ada permintaan untuk stop operasi
            if (request_to_stop)
            {
                logger.info("Stopping operation");
                if (motion_cancel())
                {
                    fsm_robot.value = FSM_SAFEOP;
                }
                break;
            }

            // Jika baterai mau habis
            if (battery_soc < threshold_minimum_battery_soc)
            {
                logger.warn("Battery SOC is below threshold, switching to charging mode");
                if (motion_cancel())
                {
                    fsm_robot.value = FSM_DOING_CHARGING;
                }
                break;
            }

            // Safety over index
            if (current_route_index >= route_list.size())
            {
                logger.error("Current route index exceeds route list size, resetting to 0");
                current_route_index = 0;
            }

            // Jalan ke titik
            if (motion_to_point(route_list[current_route_index].target_pose_x,
                                route_list[current_route_index].target_pose_y,
                                route_list[current_route_index].target_pose_yaw,
                                1.0, 0.5))
            {
                logger.info("Reached target point: %d", current_route_index);
                current_route_index++;
            }
            break;
        }

        case FSM_DOING_CHARGING:
        {
            break;
        }

        case FSM_TEST_MULTIROBOT: {
            basestation_manager();
        }
        }

        global_counter_ms += 20;
        ds4_previous_status = ds4_current_status;
    }
    
    void ds4_handler() {
        if(fabsf(ds4_current_status.axis_left_y) > 0.1f || fabsf(ds4_current_status.axis_right_x) > 0.1f)
        {
            counter_stop_command = 0;

            if (robot_current_mode == 2) {
                logger.warn("robot_operation_status set to 0");
                robot_operation_status = 0;
            }
            
            float vx = ds4_current_status.axis_left_y * max_vx;
            float vth = ds4_current_status.axis_right_x * max_vx;
            send_speed_command(vx, vth);
        }
        else if (robot_operation_status == 0){
            send_stop_command();
        }

        if (ds4_current_status.button_cross > 0 && ds4_previous_status.button_cross == 0 && robot_current_mode == 2) {
            robot_operation_status = 1;
        }

        if (ds4_current_status.axis_l2 > 0 &&  ds4_previous_status.axis_l2 == 0) {
            if (robot_current_mode != 2) {
                robot_operation_status = 3;
            } else {
                robot_operation_status = 2;
            }
        }

        if (ds4_current_status.button_circle > 0 && ds4_previous_status.button_circle == 0 && robot_current_mode == 2) {
            robot_operation_status = 4;
        }
    }

    void mapping_mode() {
        static uint8_t enable_mapping_mode = 1;
        if (enable_mapping_mode > 0) {
            printf("========================================================\n");
            logger.info("Robot Set to Mapping Mode %d", set_mapping_mode);
            logger.info("Move around then press R2 to save the map");
            cpr::Response res = cpr::Post(cpr::Url(build_url(ENDPOINT_SET_MODE)), 
                                                cpr::Body("{\"mode\":" + std::to_string(set_mapping_mode) + "}"),
                                                cpr::Header{{"Content-Type", "application/json"}});
            
            while (robot_current_mode > 1) {
                res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_MODE)));
                nlohmann::json json_res = nlohmann::json::parse(res.text);
                robot_current_mode = json_res["mode"];

                sleep(0.100);
            }

            printf("========================================================\n");
            enable_mapping_mode = 0;
        }
        
        logger.info("MAPPING MODE ENABLED");
        if (ds4_current_status.axis_r2 > 0 && ds4_previous_status.axis_r2 == 0) {
            try {
                cpr::Response res = cpr::Post(cpr::Url(build_url(ENDPOINT_SAVE_MAP)), 
                                                cpr::Body("{}"),
                                                cpr::Header{{"Content-Type", "application/json"}});
                                                
                printf("========================================================\n");
                logger.warn("Map Saved!!");
                
                res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_CURRENT_MAP_NAME)));
                std::cout << res.text << std::endl;

                res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_MAP_LISTS)));
                std::cout << res.text << std::endl;

                // json_res = nlohmann::json::parse(res.text);
                // logger.info("Available map Lists: ");

                // for (const auto& item : json_res) {
                //     logger.info("Map Name: %s", item["name"]);
                // }

                printf("========================================================\n");
            } catch (const std::exception &e) {
                logger.error("Error saving map: %s", e.what());
            }
            
            robot_operation_status = 3;
            enable_mapping_mode = 1;
        }
    }
    
    void operation_manager() {
        switch (robot_operation_status) {
            case 0:
                printf("Robot operation status: 0\n");
                break;
            case 1:
                if (motion_to_point(-1, 0, 0))
                {
                    // robot_operation_status = 11;
                }
                break;
            case 11:
                if (motion_to_point(2.35, 3, 1.61))
                {
                    // printf("Change to 12\n");
                    robot_operation_status = 12;
                }
                break;
            case 12:
                if (motion_to_point(1.5, 3, 1.61))
                {
                    // printf("Change to 13\n");
                    robot_operation_status = 13;
                }
                break;
            case 13:
                if (motion_to_point(1.5, 2, 1.61))
                {
                    // printf("Change to 14\n");
                    robot_operation_status = 14;
                }
                break;
            case 14:
                if (motion_to_point(2.35, 2, 1.61))
                {
                    // printf("Change to 0\n");
                    robot_operation_status = 0;
                }
                break;
            case 2:
                mapping_mode();
                break;
            case 3:
                {
                    cpr::Response res = cpr::Post(cpr::Url(build_url(ENDPOINT_SET_MODE)),
                                                    cpr::Body("{\"mode\":2}"),
                                                    cpr::Header{{"Content-Type", "application/json"}});
                    robot_operation_status = 0;
                    printf("========================================================\n");
                    logger.info("Switching to Navigation Mode.........");
                    printf("========================================================\n");

                    while (robot_current_mode < 2) {
                        res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_MODE)));
                        nlohmann::json json_res = nlohmann::json::parse(res.text);
                        robot_current_mode = json_res["mode"];

                        sleep(0.100);
                    }
                    break;
                }
            case 4:
                logger.info("Following Set Up Routes");
                motion_follow_routes();
                break;
            // default:
            //     send_stop_command();
            //     break;
            
        }
    }

    void motion_follow_routes() {
        static uint8_t change_terminals = 1;

        if (current_robot_config.current_route_index >= current_robot_config.routes.size()) {
            robot_operation_status = 0;
            current_robot_config.current_route_index = 0;
            return;
        }

        if (change_terminals > 0) {
            for (const auto& terminal : current_terminals) {
                if (terminal.terminal_id == current_robot_config.routes[current_robot_config.current_route_index]) {
                    current_robot_config.target_pose_x = terminal.target_pose_x;
                    current_robot_config.target_pose_y = terminal.target_pose_y;
                    current_robot_config.target_pose_theta = terminal.target_pose_theta;

                    current_robot_config.max_velocity_x = terminal.target_max_velocity_x;
                    current_robot_config.max_velocity_y = terminal.target_max_velocity_y;
                    current_robot_config.max_velocity_theta = terminal.target_max_velocity_theta;

                    current_robot_config.position_error_tolerance = terminal.toleransi_posisi;
                    current_robot_config.angle_error_tolerance = terminal.toleransi_sudut;

                    current_robot_config.current_terminal = terminal.terminal_id;
                    if (current_robot_config.current_route_index + 1 < current_robot_config.routes.size()) {
                        current_robot_config.next_terminal = current_robot_config.routes[current_robot_config.current_route_index + 1];
                    } else {
                        current_robot_config.next_terminal = 0;
                    }
                    break;
                }
            }
        }

        logger.info("Current Target Pose: x=%.2f, y=%.2f, theta=%.2f",
                    current_robot_config.target_pose_x,
                    current_robot_config.target_pose_y,
                    current_robot_config.target_pose_theta);
        logger.info("Current Max Velocity: vx=%.2f, vy=%.2f, vtheta=%.2f",
                                current_robot_config.max_velocity_x,
                                current_robot_config.max_velocity_y,
                                current_robot_config.max_velocity_theta);
        logger.info("Current Route Index: %d", current_robot_config.current_route_index);
        logger.info("Current Route Size: %d", current_robot_config.routes.size());
        logger.info("Current Position Error Tolerance: %.2f", current_robot_config.position_error_tolerance);
        logger.info("Current Terminal ID: %d Next Terminal ID: %d", current_robot_config.current_terminal, current_robot_config.next_terminal);

        if (motion_to_point(current_robot_config.target_pose_x, current_robot_config.target_pose_y, current_robot_config.target_pose_theta,
                current_robot_config.max_velocity_x, current_robot_config.position_error_tolerance, current_robot_config.angle_error_tolerance))
        {
            change_terminals = 1;
            current_robot_config.current_route_index++;
        }
    }

    void load_terminals()
    {
        std::ifstream fin;
        std::string line;
        std::vector<std::string> tokens;
        bool is_terminal_loaded_normally = false;

        all_terminal.terminals.clear();
        current_terminals.clear();

        try
        {
            fin.open(terminals_path, std::ios::in);
            if (fin.is_open())
            {
                is_terminal_loaded_normally = true;
                while (std::getline(fin, line))
                {
                    if (line.find("type") != std::string::npos)
                    {
                        continue;
                    }
                    boost::split(tokens, line, boost::is_any_of(","));

                    for (auto &token : tokens)
                    {
                        boost::trim(token);
                    }

                    terminal_t terminal;
                    terminal.terminal_type = std::stoi(tokens[0]);
                    terminal.terminal_id = std::stoi(tokens[1]);
                    terminal.target_pose_x = std::stof(tokens[2]);
                    terminal.target_pose_y = std::stof(tokens[3]);
                    terminal.target_pose_theta = std::stof(tokens[4]);
                    terminal.target_max_velocity_x = std::stof(tokens[5]);
                    terminal.target_max_velocity_y = std::stof(tokens[6]);
                    terminal.target_max_velocity_theta = std::stof(tokens[7]);
                    terminal.toleransi_posisi = std::stof(tokens[8]);
                    terminal.toleransi_sudut = std::stof(tokens[9]);
                    current_terminals.push_back(terminal);

                    ros2_interface::msg::Terminal terminal_buffer;
                    terminal_buffer.terminal_type = std::stoi(tokens[0]);
                    terminal_buffer.terminal_id = std::stoi(tokens[1]);
                    terminal_buffer.target_pose_x = std::stof(tokens[2]);
                    terminal_buffer.target_pose_y = std::stof(tokens[3]);
                    terminal_buffer.target_pose_theta = std::stof(tokens[4]);
                    terminal_buffer.target_max_speed = std::stof(tokens[5]);
                    terminal_buffer.target_max_velocity_x = std::stof(tokens[5]);
                    terminal_buffer.target_max_velocity_y = std::stof(tokens[6]);
                    terminal_buffer.target_max_velocity_theta = std::stof(tokens[7]);
                    terminal_buffer.toleransi_posisi = std::stof(tokens[8]);
                    terminal_buffer.toleransi_sudut = std::stof(tokens[9]);
                    
                    all_terminal.terminals.push_back(terminal_buffer);
                    
                    tokens.clear();
                }
                fin.close();

                logger.info("Terminal file loaded");
            }

            if (fin.fail() && !fin.is_open() && !is_terminal_loaded_normally)
            {
                logger.warn("Failed to load terminal file, Recreate the terminal file");
                save_terminals(all_terminal);
            }
        }
        catch (const std::exception &e)
        {
            logger.error("Failed to load terminal file: %s, Recreate the terminal file", e.what());
            save_terminals(all_terminal);
        }
    }

    void save_terminals(ros2_interface::msg::TerminalArray available_terminals)
    {
        std::ofstream fout;

        try
        {
            fout.open(terminals_path, std::ios::out);
            if (fout.is_open())
            {
                fout << "type, id, x, y, theta, max_vx, max_vy, max_vtheta, toleransi_posisi, toleransi_sudut" << std::endl;
                for (auto terminal : available_terminals.terminals)
                {
                    int terminal_type_integer = terminal.terminal_type;
                    int terminal_id_integer = terminal.terminal_id;
                    fout << terminal_type_integer << ", " << terminal_id_integer << ", " << terminal.target_pose_x << ", " << terminal.target_pose_y << ", " << terminal.target_pose_theta << ", " << terminal.target_max_velocity_x << ", " << terminal.target_max_velocity_y << ", " << terminal.target_max_velocity_theta << ", " << terminal.toleransi_posisi << ", " << terminal.toleransi_sudut << std::endl;
                }
                fout.close();

                logger.info("Terminal file saved");
            }
        }
        catch (const std::exception &e)
        {
            logger.warn("Failed to open terminal file, Recreate the terminal file");
        }
    }

    void load_robot_config()
    {
        std::ifstream fin;
        std::string line;
        std::vector<std::string> tokens;
        bool is_robot_loaded_normally = false;

        all_robot.robots.clear();

        try
        {
            fin.open(robot_config_path, std::ios::in);
            if (fin.is_open())
            {
                is_robot_loaded_normally = true;
                while (std::getline(fin, line))
                {
                    if (line.find("ip") != std::string::npos)
                    {
                        continue;
                    }
                    boost::split(tokens, line, boost::is_any_of(","));

                    for (auto &token : tokens)
                    {
                        boost::trim(token);
                    }

                    current_robot_config.ip_addr = tokens[0];
                    current_robot_config.init_pose_x = std::stof(tokens[1]);
                    current_robot_config.init_pose_y = std::stof(tokens[2]);
                    current_robot_config.init_pose_theta = std::stof(tokens[3]);
                    
                    ros2_interface::msg::Robot robot_single;
                    robot_single.ip_addr = tokens[0];                    
                    robot_single.init_pose_x = std::stof(tokens[1]);
                    robot_single.init_pose_y = std::stof(tokens[2]);
                    robot_single.init_pose_theta = std::stof(tokens[3]);

                    std::vector<std::string> routes_tokens;
                    boost::split(routes_tokens, tokens[4], boost::is_any_of("-"));
                    for (auto token : routes_tokens)
                    {
                        int16_t route = std::stoi(token);
                        current_robot_config.routes.push_back(route);
                        robot_single.routes.push_back(route);
                    }
                    
                    
                    std::vector<std::string> routes_init_tokens;
                    boost::split(routes_init_tokens, tokens[5], boost::is_any_of("-"));
                    for (auto token : routes_init_tokens)
                    {
                        int16_t route_init = std::stoi(token);
                        robot_single.routes_init.push_back(route_init);
                        current_robot_config.routes_init.push_back(route_init);
                    }
                    
                    robot_single.current_route_index = 0;
                    current_robot_config.current_route_index = 0;

                    all_robot.robots.push_back(robot_single);
                    tokens.clear();
                }
                fin.close();

                logger.info("Robot file loaded");
            }

            if (fin.fail() && !fin.is_open() && !is_robot_loaded_normally)
            {
                logger.warn("Failed to load robot file, Recreate the robot file");
                save_robot_config();
            }
        }
        catch (const std::exception &e)
        {
            logger.error("Failed to load robot file: %s", e.what());
        }
    }

    void save_robot_config()
    {
        std::ofstream fout;

        try
        {
            fout.open(robot_config_path, std::ios::out);
            if (fout.is_open())
            {
                fout << "ip, init_x, init_y, init_theta, routes, routes_init" << std::endl;
                logger.info("Saving robot: %scurrent_", current_robot_config.ip_addr.c_str());
                fout << current_robot_config.ip_addr << ", " << current_robot_config.init_pose_x << ", " << current_robot_config.init_pose_y << ", " << current_robot_config.init_pose_theta << ", ";

                for (auto route : current_robot_config.routes)
                {
                    int route_integer = route;
                    if (route == current_robot_config.routes.back())
                    {
                        fout << route_integer;
                    }
                    else
                    {
                        fout << route_integer << "-";
                    }
                }
                fout << ", ";

                for (auto route_init : current_robot_config.routes_init)
                {
                    int route_init_integer = route_init;
                    if (route_init == current_robot_config.routes_init.back())
                    {
                        fout << route_init_integer;
                    }
                    else
                    {
                        fout << route_init_integer << "-";
                    }
                }
                fout << std::endl;
                fout.close();

                logger.info("Robot file saved");
            }
        }
        catch (const std::exception &e)
        {
            logger.warn("Failed to open terminal file, Recreate the terminal file");
        }
    }

    // MULTIROBOT LOGIC
    void basestation_manager() {
        transmit_all();

        for (auto& robot : all_robot.robots) {
            switch (robot.machine_state) {
                case 1:
                    motion_follow_routes(&robot);
                    break;
                default:
                    logger.info("Robot %s Case default", robot.ip_addr);
                    break;
            }
        }

        bs_reload_process = 0;
    }

    ros2_interface::msg::Terminal *find_terminal_by_id(int terminal_id)
    {
        for (size_t i = 0; i < all_terminal.terminals.size(); i++)
        {
            if (all_terminal.terminals[i].terminal_id == terminal_id)
            {
                return &all_terminal.terminals[i];
            }
        }

        static ros2_interface::msg::Terminal terminal;
        terminal.terminal_id = -1;
        return &terminal;
    }

    bool motion_to_point(ros2_interface::msg::Robot *robot, float target_x, float target_y, float target_yaw, float max_speed = 0.3, float position_error_tolerance = 0.5, float angle_error_tolerance = 0.5) {
        if (robot->mode != 2) {
            logger.error("Robot %s is not in navigation mode, cannot move to point", robot->ip_addr);
            return false;
        }

        static std::vector<std::pair<std::string, uint8_t>> global_motion_to_point_status;
        if (bs_reload_process > 0) {
            global_motion_to_point_status.clear();
            for (const auto& robot_buffer : all_robot.robots) {
                global_motion_to_point_status.emplace_back(robot_buffer.ip_addr, 0);
            }
        }

        for (auto& mtp_status : global_motion_to_point_status) {
            if (mtp_status.first == robot->ip_addr) {
                if (mtp_status.second == 0) {
                    cpr::Response res = cpr::Post(cpr::Url(build_url(ENDPOINT_SET_MAX_TRANS_SPEED, robot->ip_addr)),
                                        cpr::Body("{\"speed\":" + std::to_string(max_speed) + "}"),
                                        cpr::Header{{"Content-Type", "application/json"}});
                    res = cpr::Post(cpr::Url(build_url(ENDPOINT_GO2TARGET, robot->ip_addr)),
                                              cpr::Body("{\"x\":" + std::to_string(target_x) + ",\"y\":" + std::to_string(target_y) + ",\"theta\":" + std::to_string(target_yaw) + "}"),
                                              cpr::Header{{"Content-Type", "application/json"}});
        
                    mtp_status.second = 1;
                } else if (mtp_status.second == 1) {
                    float target_error_position = sqrtf(powf(robot->current_pose_x - target_x, 2) + powf(robot->current_pose_y - target_y, 2));
                    float target_error_angular = fabsf(robot->current_pose_theta - target_yaw);

                    if (target_error_position < position_error_tolerance && target_error_angular < angle_error_tolerance) {
                        mtp_status.second = 0;
                        return true; 
                    } 
                }
            }
        }
        
        return false;
    }

    void motion_follow_routes(ros2_interface::msg::Robot *robot) {
        static std::vector<std::pair<std::string, uint8_t>> global_change_target;
        if (bs_reload_process > 0) {
            global_change_target.clear();
            for (const auto& robot_buffer : all_robot.robots) {
                global_change_target.emplace_back(robot_buffer.ip_addr, 1);
            }
        }
        
        if (robot->current_route_index >= robot->routes.size()) {
            robot->current_route_index = 0;
            return;
        }

        for (auto& change_target : global_change_target) {
            ros2_interface::msg::Terminal *current_terminal = find_terminal_by_id(robot->terminal_id_current);

            if (change_target.first == robot->ip_addr && change_target.second > 0) {
                current_terminal = find_terminal_by_id(robot->routes[robot->current_route_index]);

                robot->target_pose_x = current_terminal->target_pose_x;
                robot->target_pose_y = current_terminal->target_pose_y;
                robot->target_pose_theta = current_terminal->target_pose_theta;

                robot->max_velocity_x = current_terminal->target_max_velocity_x;
                robot->max_velocity_y = current_terminal->target_max_velocity_y;
                robot->max_velocity_theta = current_terminal->target_max_velocity_theta;

                robot->terminal_id_previous = robot->terminal_id_current;
                robot->terminal_id_current = current_terminal->terminal_id;

                if (robot->current_route_index + 1 < robot->routes.size()) {
                    robot->terminal_id_next = robot->routes[robot->current_route_index + 1];
                } else {
                    robot->terminal_id_next = -99;
                }

                change_target.second = 0;
            }

            if (motion_to_point(robot, robot->target_pose_x, robot->target_pose_y, robot->target_pose_theta,
                    robot->max_velocity_x, current_terminal->toleransi_posisi, current_terminal->toleransi_sudut))
            {
                change_target.second = 1;
                robot->current_route_index++;
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_io_reeman = std::make_shared<IOReeman>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_io_reeman);
    executor.spin();

    return 0;
}