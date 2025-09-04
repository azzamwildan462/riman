string ip_addr

float32 init_pose_x
float32 init_pose_y 
float32 init_pose_theta

float32 current_pose_x
float32 current_pose_y
float32 current_pose_theta 
float32 current_velocity_x 
float32 current_velocity_y 
float32 current_velocity_theta 

float32 target_pose_x 
float32 target_pose_y 
float32 target_pose_theta 
float32 target_velocity_x 
float32 target_velocity_y 
float32 target_velocity_theta 

float32 incremental_odometry

float32 max_velocity_x
float32 max_velocity_y 
float32 max_velocity_theta 

int8 machine_state 
int16 current_route_init_index
int16 current_route_index

int16 current_terminal 
int16 next_terminal 
int16 blocked_on

int16[] routes_init
int16[] routes

int32 error_code 

string command
string prev_command

int16 fb_command

bool has_finished_routes_init

int8 mode

float32 battery_soc
int8 charge_flag
int8 emergency_button

int8 nav_state
int8 nav_code
float32 dist_to_goal
float32 mileage

int16 terminal_id_current
int16 terminal_id_next
int16 terminal_id_previous



