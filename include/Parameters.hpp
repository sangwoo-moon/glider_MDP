#pragma once
#include <string>

// Terrain map file location
extern std::string g_param_terrain_map_file;

// Thermal map file location
extern std::string g_param_thermal_map_file;

// Geofence files directory location 
extern std::string g_param_allowed_territories_dir;

// Airspeed values
extern float *g_param_airspeeds;
extern float g_param_Thermik_airspeeds[9];
extern float g_param_Radian_airspeeds[4];

// Polar-Curve values
extern float *g_param_curve;
extern float g_param_Thermik_curve[3];
extern float g_param_Radian_curve[3];

// Time to wait for an acknowledgement of an issued command from the sailplane, in milliseconds
extern unsigned int g_param_ack_wait_timeout;

// Duration of a glide action, in seconds
extern unsigned int g_param_glide_action_duration;

// The sailplane's planning horizon, in seconds
extern unsigned int g_param_planning_horizon;

// Time for planner to sleep during each action's execution, in seconds
extern unsigned int g_param_planning_sleep;

// Number of receding horizon steps (?)
extern unsigned int g_param_receding_horiz_values;

// Height AGL below which we don't want the sailplane to descend, in meters
extern unsigned int g_param_alt_slack;

// Maximum altitude AGL above which we don't want the sailplane to fly, in meters
extern unsigned int g_param_max_alt;

// Altitude AGL at which thermal activity stops, in meters
extern unsigned int g_param_inversion_ceiling;

// Map buffer
extern float g_param_map_buffer;

// Home Location for Planner (Not for Mavlink/Pixhawk!)
extern float *g_home;
extern float g_home_airport[2];
extern float g_home_lakeside[2];

// Airspeed command when sailplane starts soaring
extern float g_param_aspd_soar;
extern float g_param_aspd_soar_Radian;
extern float g_param_aspd_soar_Thermik;