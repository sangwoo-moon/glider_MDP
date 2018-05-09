#pragma once
#include "MavLinkSailplane.hpp"
#include "TerrainMap.hpp"
#include "ThermalPredictionModel.hpp"
#include "Geofence.hpp"
#include <iostream>

using namespace mavlinkcom;


struct GlobalPosition {
	// (longitude, lattitude) coordinates w.r.t. home local position
	float lat;
	float lon;
};

struct Altitude {
	// altitudes w.r.t terrain and home position are in meters
	float wrt_terrain;
	float wrt_msl;
};


struct Attitude {
	// pitch and yaw attitude
	float pitch;
	float yaw;
};

class State
{
private:

	static std::shared_ptr<MavLinkSailplane> s_vehicle;
	static std::shared_ptr<TerrainMap> s_terrain_map;
	static std::shared_ptr<ThermalPredictionModel> s_thermal_prediction_model;
	static std::shared_ptr<Geofence> s_geofence_map;

	GlobalPosition _global_pos;
	Altitude _altitude;
	Attitude _attitude;

	// airspeed in m/s
	float _airspeed;

public:

	State();
	~State();
	static void initialize(std::shared_ptr<MavLinkSailplane> vehicle, std::shared_ptr<TerrainMap> terrain_map, 
		std::shared_ptr<Geofence> geofence_map, std::shared_ptr<ThermalPredictionModel> thermal_prediction_model);
	bool get_curr_state();
	static void get_curr_state(State* pState);
	void copy_state(State* pState);

	GlobalPosition* get_global_pos();
	Altitude* get_altitude();
	Attitude* get_attitude();
	float* get_airspeed();

	void set_global_pos(GlobalPosition *global_pos);
	void set_altitude(Altitude *altitude);
	void set_attitude(Attitude *attitude);
	void set_airspeed(float *airspeed);

	static std::shared_ptr<MavLinkSailplane> get_vehicle();
	static std::shared_ptr<TerrainMap> get_terrain_map();
	static std::shared_ptr<ThermalPredictionModel> get_thermal_prediction_model();
	static std::shared_ptr<Geofence> get_geofence_map();

	void print(std::ostream& out);
};