#include "State.hpp"
#include "Utils.hpp"
#include <new>


std::shared_ptr<MavLinkSailplane> State::s_vehicle = nullptr;
std::shared_ptr<TerrainMap> State::s_terrain_map = nullptr;
std::shared_ptr<ThermalPredictionModel> State::s_thermal_prediction_model = nullptr;
std::shared_ptr<Geofence> State::s_geofence_map = nullptr;


// Initialize all members to obviously invalid values to catch potential bugs due to using states with no actual data in them
State::State() :
	_global_pos({1000000.0f,1000000.0f}),	// x, y
	_altitude({-1000.0f,-1000.0f}),			// wrt_terrain, wrt_home
	_attitude({1000.0f,1000.0f}),			// pitch, yaw
	_airspeed(1000.0f)
{
}


State::~State()
{
}


void
State::initialize(std::shared_ptr<MavLinkSailplane> vehicle, std::shared_ptr<TerrainMap> terrain_map, 
	std::shared_ptr<Geofence> geofence_map, std::shared_ptr<ThermalPredictionModel> thermal_prediction_model)
{
	s_vehicle = vehicle;
	s_terrain_map = terrain_map;
	s_thermal_prediction_model = thermal_prediction_model;
	s_geofence_map = geofence_map;
}

bool
State::get_curr_state()
{

	if (s_vehicle != nullptr)
	{
		// set the member variables of pState to values retrieved from vehicle
		// retrieve all values of interest from _vehicle at once (use locks if necessary) to avoid
		// getting inconsistent states

		float *elev = (float *)malloc(sizeof(float));

		_global_pos.lat =	s_vehicle->getVehicleState().global_est.pos.lat;
		_global_pos.lon =	s_vehicle->getVehicleState().global_est.pos.lon;

		// X-plane shows MSL based altitude. check whether PixHwak gives in a same way.
		_altitude.wrt_msl = s_vehicle->getVehicleState().global_est.pos.alt;

		//s_vehicle->getVehicleState().altitude.altitude_terrain is not updated at the MavLink
		s_terrain_map->get_elevation(_global_pos.lat, _global_pos.lon, elev);
		_altitude.wrt_terrain =	_altitude.wrt_msl - *elev;
		
		_attitude.pitch =	s_vehicle->getVehicleState().attitude.pitch;
		_attitude.yaw =		s_vehicle->getVehicleState().attitude.yaw;

		_airspeed =			s_vehicle->getVehicleState().vfrhud.true_airspeed;
		
		free(elev);
		return true;
	}

	return false;
}


void
State::copy_state(State* pState)
{
	// set the member variables of pState
	// retrieve all values of interest from _vehicle at once 

	if (pState != nullptr)
	{
		_global_pos.lat = pState->_global_pos.lat;
		_global_pos.lon = pState->_global_pos.lon;

		_altitude.wrt_msl = pState->_altitude.wrt_msl;
		_altitude.wrt_terrain = pState->_altitude.wrt_terrain;

		_attitude.pitch = pState->_attitude.pitch;
		_attitude.yaw = pState->_attitude.yaw;

		_airspeed = pState->_airspeed;

	}
}


void
State::get_curr_state(State* pState)
{
	// set the member variables of pState to values retrieved from vehicle
	// retrieve all values of interest from _vehicle at once (use locks if necessary) to avoid
	// getting inconsistent states

	if (pState != nullptr)
	{
		pState->_global_pos.lat = 0;
		pState->_global_pos.lon = 0;

		pState->_altitude.wrt_msl = 0;
		pState->_altitude.wrt_terrain = 0;

		pState->_attitude.pitch = 0;
		pState->_attitude.yaw = 0;

		pState->_airspeed = 0;

	}
}


GlobalPosition* State::get_global_pos() { return &_global_pos; }
Altitude* State::get_altitude() { return &_altitude; }
Attitude* State::get_attitude() { return &_attitude; }
float* State::get_airspeed() { return &_airspeed; }
std::shared_ptr<MavLinkSailplane> State::get_vehicle() { return s_vehicle; }
std::shared_ptr<TerrainMap> State::get_terrain_map() { return s_terrain_map; }
std::shared_ptr<ThermalPredictionModel> State::get_thermal_prediction_model() { return s_thermal_prediction_model; }
std::shared_ptr<Geofence> State::get_geofence_map() { return s_geofence_map; }

void State::set_global_pos(GlobalPosition *global_pos) { _global_pos = *global_pos; }
void State::set_altitude(Altitude *altitude) { _altitude = *altitude; }
void State::set_attitude(Attitude *attitude) { _attitude = *attitude; }
void State::set_airspeed(float *airspeed) { _airspeed = *airspeed; }


void 
State::print(std::ostream& out)
{
	// print a description of this state to ostream
	// printf("Lat: %3.6f, Lon: %3.6f, Yaw: %3.2f, MSL Altitude: %4.2f, AGL Altitude: %4.2f\n", _global_pos.lat, _global_pos.lon, _attitude.yaw/D2R, _altitude.wrt_msl, _altitude.wrt_terrain);
	// printf("lat\t%3.6f\tlon\t%3.6f\thead\t%3.2f\tMSL\t%4.2f\tAGL\t%4.2f\n", _global_pos.lat, _global_pos.lon, _attitude.yaw / D2R, _altitude.wrt_msl, _altitude.wrt_terrain);

    out << "lat\t" << std::fixed << std::setprecision(9) << _global_pos.lat << "\tlon\t" << std::fixed << std::setprecision(9) << _global_pos.lon << std::fixed << std::setprecision(9) << "\thead\t" << std::fixed << std::setprecision(5) << _attitude.yaw / D2R << "\tMSL\t" << std::fixed << std::setprecision(6) << _altitude.wrt_msl << "\tAGL\t" << std::fixed << std::setprecision(6) << _altitude.wrt_terrain << std::endl;

}