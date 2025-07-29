#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/simple_fsm.hpp"

#include "communication/nlohmann_json.hpp"
#include "cpr/cpr.h"

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

#define FSM_INIT 0
#define FSM_SAFEOP 1
#define FSM_OPERATION 2
#define FSM_DOING_CHARGING 3

typedef struct
{
    float target_pose_x;
    float target_pose_y;
    float target_pose_yaw;
} route_t;

class IOReeman : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    HelpLogger logger;
    MachineState fsm_robot;

    // Configs
    std::string reeman_controller_ip = "10.7.101.102";
    int threshold_minimum_request_period_ms = 1000; // Minimum period between requests in milliseconds
    float threshold_minimum_battery_soc = 20;

    // Robot vars
    float final_pose_x = 0.0f;
    float final_pose_y = 0.0f;
    float final_pose_yaw = 0.0f;
    float battery_soc = 0.0f;
    uint8_t emergency_button = 0;
    std::vector<route_t> route_list;
    uint8_t current_route_index = 0;

    // Sesuatu
    uint64_t global_counter_ms = 0;
    uint64_t last_time_request_counter_ms = 0;
    bool request_to_start = false;
    bool request_to_stop = false;

    IOReeman()
        : Node("io_reeman")
    {
        this->declare_parameter("reeman_controller_ip", reeman_controller_ip);
        this->get_parameter("reeman_controller_ip", reeman_controller_ip);

        this->declare_parameter("threshold_minimum_request_period_ms", threshold_minimum_request_period_ms);
        this->get_parameter("threshold_minimum_request_period_ms", threshold_minimum_request_period_ms);

        this->declare_parameter("threshold_minimum_battery_soc", threshold_minimum_battery_soc);
        this->get_parameter("threshold_minimum_battery_soc", threshold_minimum_battery_soc);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        // fsm_robot.value = FSM_SAFEOP;
        fsm_robot.value = FSM_INIT;

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&IOReeman::callback_routine, this));

        logger.info("IOReeman node initialized");
    }

    ~IOReeman()
    {
    }

    // ===================================================================================================================

    bool do_request()
    {
        if (global_counter_ms - last_time_request_counter_ms < threshold_minimum_request_period_ms)
        {
            logger.warn("Skipping request, not enough time has passed since the last request");
            return false;
        }
    }

    std::string build_url(const std::string &endpoint)
    {
        if (reeman_controller_ip.empty())
        {
            logger.error("Reeman controller IP is not set");
            return "";
        }
        return "http://" + reeman_controller_ip + endpoint;
    }

    void get_route_lists()
    {
    }

    // ===================================================================================================================

    void get_motion_nav_status(uint16_t *nav_status, float *error_position_m)
    {
    }

    bool motion_to_point(float target_x, float target_y, float target_yaw, float v_trans_max, float v_rot_max, float error_tolerance = 0.5)
    {
    }

    bool motion_cancel()
    {
        std::string ep_cancel = build_url(ENDPOINT_CANCEL_GO2TARGET);
    }

    // ===================================================================================================================

    void fetchData() {
        cpr::Response res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_POSE)));
        try {
            nlohmann::json json_response;
            json_response = nlohmann::json::parse(res.text);
    
            final_pose_x = json_response["x"];
            final_pose_y = json_response["y"];
            final_pose_yaw = json_response["theta"];
            
            res = cpr::Get(cpr::Url(build_url(ENDPOINT_GET_PWR_MGMT)));
            json_response = nlohmann::json::parse(res.text);
            emergency_button = json_response["emergencyButton"];
    
            printf("Final Pose: x=%.2f, y=%.2f, yaw=%.2f\n", final_pose_x, final_pose_y, final_pose_yaw);
            printf("Emergency Button: %d\n", emergency_button);
        } catch (const std::exception &e) {
            logger.error("Error fetching Data: %s", e.what());
            return;
        }
    }

    void callback_routine()
    {
        switch (fsm_robot.value)
        {
        case FSM_INIT:
        {
            fetchData();
            break;
        }

        case FSM_SAFEOP:
        {
            if (request_to_start)
            {
                if (route_list.size() > 0)
                {
                    fsm_robot.value = FSM_OPERATION;
                }
            }
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
        }

        global_counter_ms += 20;
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