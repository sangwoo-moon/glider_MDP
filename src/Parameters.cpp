#include "Parameters.hpp"


// Terrain map loading
// for Carnation
std::string g_param_terrain_map_file = "carnation_MainMap_rows_109_cols_106_mpp_10.000000_nlat_47.687789_slat_47.677986_elon_-121.943700_wlon_-121.957861.csv"; // "imgn39w119_13_rows_10812_cols_10812_mpp_10_nlat_039.00055_slat_037.99944_elon_ - 117.99944_wlon_ - 119.00055.csv";			
// for Hawthorne Airport
//std::string g_param_terrain_map_file = "hawthorne_airport_rows_67_cols_68_mpp_10.307360_nlat_38.554535_slat_38.548313_elon_-118.628906_wlon_-118.637079.csv";
// for Hawthorne Lakeside
//std::string g_param_terrain_map_file = "hawthorne_lakeside_rows_688_cols_397_mpp_10.307360_nlat_38.668329_slat_38.604550_elon_-118.597174_wlon_-118.644358.csv";

// Thermal map file location
// for Carnation
std::string g_param_thermal_map_file = "carnation_ThermalModel_rows_109_cols_106_mpp_10.000000_nlat_47.687789_slat_47.677986_elon_-121.943700_wlon_-121.957861.csv";//"thrmal39w119_13_rows_10812_cols_10812_mpp_10_nlat_039.00055_slat_037.99944_elon_-117.99944_wlon_-119.00055.csv";
// for Hawthorne Airport
//std::string g_param_thermal_map_file = "hawthorne_ThermalMap_rows_67_cols_68_mpp_10.307360_nlat_38.554535_slat_38.548313_elon_-118.628906_wlon_-118.637079.csv";
// for Hawthorne Lakeside
//std::string g_param_thermal_map_file = "hawthorne_thermalmap_rows_688_cols_397_mpp_10.307360_nlat_38.668329_slat_38.604550_elon_-118.597174_wlon_-118.644358.csv";

// Geofence files directory location
std::string g_param_allowed_territories_dir = "AllowedTerritories\\";

// Airspeed values
// for Thermik
float g_param_Thermik_airspeeds[] = { 13.5, 14, 15, 16, 17, 19, 21, 23, 25 }; // { 12, 12.5, 13, 13.5, 14, 15, 16, 17, 19, 21, 23, 25 };
// for Radian
float g_param_Radian_airspeeds[] = { 8.5, 9, 9.5, 10 };
 
float *g_param_airspeeds = g_param_Radian_airspeeds;

// Curve coefficients   {sink rate} = {CURVE_A}x{airspeed}^2 + {CURVE_B}x{airspeed} + {CURVE_C}
// for Thermik
float g_param_Thermik_curve[] = { -0.01518, 0.34008, -2.33233 };
// for Radian
float g_param_Radian_curve[] = { -0.03099261, 0.44731854, -2.30292972 };

float *g_param_curve = g_param_Radian_curve;

// Time to wait for an acknowledgement of an issued command from the sailplane, in milliseconds
unsigned int g_param_ack_wait_timeout = 4000;

//* Duration of a glide action, in seconds
unsigned int g_param_glide_action_duration = 20;

// The sailplane's planning horizon, in seconds
unsigned int g_param_planning_horizon = 10;

//* Time for the planner to sleep during each action's execution, in seconds
unsigned int g_param_planning_sleep = 15;

//* Number of receding horizon steps
unsigned int g_param_receding_horiz_values = 10;

// Height AGL below which we don't want the sailplane to descend, in meters
unsigned int g_param_alt_slack = 80;

// Maximum altitude AGL above which we don't want the sailplane to fly, in meters
unsigned int g_param_max_alt = 430;

// Altitude AGL at which thermal activity stops, in meters
unsigned int g_param_inversion_ceiling = 1800;

// Map buffer
float g_param_map_buffer = 20.0f;

// home position for planner
float g_home_airport[] = { 38.552478, -118.630757 };
float g_home_lakeside[] = { 38.638184, -118.636496 };

float *g_home = g_home_lakeside;

// Airspeed command when sailplane starts soaring
float g_param_aspd_soar_Radian = 9.0;
float g_param_aspd_soar_Thermik = 13.5;

float g_param_aspd_soar = g_param_aspd_soar_Radian;