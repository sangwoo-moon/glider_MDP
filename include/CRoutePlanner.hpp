#pragma once
#include "FileSystem.hpp"
#include "MavLinkConnection.hpp"
#include "MavLinkSailplane.hpp"
#include "Action.hpp"
#include "State.hpp"
#include "TerrainMap.hpp"
#include "Geofence.hpp"

using namespace mavlinkcom;

const unsigned g_c_num_discr_heading_values_default = 24;
const unsigned g_c_num_discr_airspeed_values_default = 12;

const float g_c_airspeed_min = 11;
const float g_c_airspeed_max = 15;
const float g_c_airspeed_opt = 12.5;
const float C_UCT = 0.5;
const unsigned ITER = 10;

const unsigned homing_wait_sec = 1;
const unsigned check_soar_period = 1;

class CRoutePlanner
{

private:

	std::shared_ptr<MavLinkSailplane> _mavlink_sailplane;
	std::shared_ptr<TerrainMap> _terrain_map;
	std::shared_ptr<ThermalPredictionModel> _thermal_prediction_model;
	std::shared_ptr<Geofence> _geofence_map;
	
	Action *_actions_all;
	Action **_actions_glide;

	unsigned _idx_action_execution;
	unsigned _num_discr_heading_values;
	unsigned _num_discr_airspeed_values;
	unsigned _num_receding_horiz_values;
	unsigned _planning_sleep;
	unsigned _reced_time;

	bool _quit;
	std::mutex _quit_mutex;

	void create_actions(unsigned num_discr_heading_values, unsigned num_discr_airspeed_values);
	void create_actions(float *discr_heading_values, unsigned num_discr_heading_values, float *discr_airspeed_values, unsigned num_discr_airspeed_values);
	void cleanup_actions();
	bool time_to_quit();


public:

	CRoutePlanner();
	CRoutePlanner(std::shared_ptr<MavLinkSailplane> mavlink_sailplane, std::shared_ptr<TerrainMap> terrain_map, 
		std::shared_ptr<Geofence> geofence_map, std::shared_ptr<ThermalPredictionModel> thermal_prediction_model,
		unsigned num_receding_horiz_values, unsigned reced_time, unsigned planning_sleep, unsigned num_discr_heading_values, unsigned num_discr_airspeed_values);
	CRoutePlanner(std::shared_ptr<MavLinkSailplane> mavlink_sailplane, std::shared_ptr<TerrainMap> terrain_map,
		std::shared_ptr<Geofence> geofence_map, std::shared_ptr<ThermalPredictionModel> thermal_prediction_model,
		unsigned num_receding_horiz_values, unsigned reced_time, unsigned planning_sleep, float *discr_heading_values, unsigned num_discr_heading_values, float *discr_airspeed_values, unsigned num_discr_airspeed_values);
	~CRoutePlanner();

    bool CRoutePlanner::initialize();
    std::shared_ptr<MavLinkSailplane> get_sailplane() { return _mavlink_sailplane; }
	bool UCT(State *state_curr, const std::shared_ptr<MavLinkSailplane> vehicle, unsigned *_idx_action_execution);
	void run();
	void quit();
	void print(std::ostream& out);

};