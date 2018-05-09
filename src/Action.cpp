#include "Action.hpp"
#include "State.hpp"
#include "math.h"
#include "Utils.hpp"
#include "Parameters.hpp"
#include <random>


using namespace mavlink_utils;


Action::Action() :
	_heading(0),
	_airspeed(OPT_GLIDE_AIRSPEED),
	_soar(false),
	_duration(DEFAULT_DURATION)
{
}


Action::Action(float heading, float airspeed, bool soar, unsigned duration) :
	_heading(heading),
	_airspeed(airspeed),
	_soar(soar),
	_duration(duration)
{
}


Action::~Action()
{
}


bool 
Action::is_feasible_in_state(State *state) const
{

	// Since actions are probabilistic, doing these checks exactly may be very expensive. Start with verifying the action's trajectory assuming wind 
	// direction and strength are deterministic, fixed at the mean.
	// {ground speed} = {airspeed} + {wind} (in vector form)
	// controllable airspeed has direction of nose of glider: should be transformed from body frame to inertia frame
	float ground_speed_x, ground_speed_y, ground_speed_z;
	float lat_next, lon_next;
	float alt_wrt_msl_next, elev_terrain_next, alt_wrt_terrain_next;
	float curr_airspeed;

	curr_airspeed = state->get_vehicle()->getVehicleState().vfrhud.true_airspeed;

	ground_speed_x			= curr_airspeed*cos(state->get_attitude()->pitch)*cos(_heading*D2R)	+ state->get_vehicle()->getVehicleState().wind.wind_x;
	ground_speed_y			= curr_airspeed*cos(state->get_attitude()->pitch)*sin(_heading*D2R)	+ state->get_vehicle()->getVehicleState().wind.wind_y;
    ground_speed_z			= curr_airspeed*sin(state->get_attitude()->pitch)					+ state->get_vehicle()->getVehicleState().wind.wind_z;

	lat_next					= state->get_global_pos()->lat				+ ground_speed_x*_duration*TIME_MARGIN / (EARTH_RADIUS*D2R);
	lon_next					= state->get_global_pos()->lon				+ ground_speed_y*_duration*TIME_MARGIN / (EARTH_RADIUS*D2R);
	alt_wrt_msl_next			= state->get_altitude()->wrt_msl			+ ground_speed_z*_duration*TIME_MARGIN;

	// Returns true iff executing this action in this state will not lead to violations of safety or legal constraints:
	//		- violating the geofence
	//		- crashing into an obstacle/terrain
	//		- whatever other constraints we come up with

	// check geofence violation
	if (!state->get_geofence_map()->is_allowed(lat_next, lon_next, alt_wrt_msl_next))
	{
		return false;
	}
		
	// check terrain violation
	/*state->get_terrain_map()->get_elevation(lat_next, lon_next, &elev_terrain_next);
	alt_wrt_terrain_next = state->get_altitude()->wrt_msl + ground_speed_z*_duration - elev_terrain_next;
	if (alt_wrt_terrain_next < 0) // UP is positive
	{
		return false;
	}
	*/
	return true;
}


bool
Action::sample_transition(State *state_curr, State *state_next, float *reward, unsigned i_step) const
{
	// This method should sample not just the next state but also the reward
	// returns true iff the state_next and reward contain valid state and reward info at the end of the method call
	if (!is_feasible_in_state(state_curr))
	{
		*reward = -1e5; // nearly infinit values for reward (negative default normal)
		return false;
	}
	else
	{
		// TAKE SAMPLING PROCEDURE AND STORE THE RESULT INTO FUTURE STATE/REWARD POINTERS.
		// sample wind
		float wind_horiz_mean, wind_vert_mean;
		float wind_horiz_var, wind_vert_var;

		float u1, u2, w, mult;	// for normal distribution random generator 
		float x1, x2;
		float wind_horiz_sampled, wind_z_sampled, wind_horiz_direction;
		float wind_x_sampled, wind_y_sampled;

		float prob;

		// set command input to current state: assume glider can follow inputs ideally (no time-delay)
		float airspeed_curr;
		Attitude attitude_curr;

		float ground_speed_x, ground_speed_y, ground_speed_z;
		GlobalPosition global_pos_next;
		Altitude altitude_next;
		float elev;

		float yaw_curr, yaw_diff;

		float strength;



		wind_horiz_mean = sqrt(pow(state_curr->get_vehicle()->getVehicleState().wind.wind_x, 2) + pow(state_curr->get_vehicle()->getVehicleState().wind.wind_y, 2));
		wind_vert_mean = state_curr->get_vehicle()->getVehicleState().wind.wind_z;

		wind_horiz_var = 0.1;
		wind_vert_var = 0.1;

		do
		{
			u1 = 2 * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 1;
			u2 = 2 * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 1;
			w = pow(u1, 2) + pow(u2, 2);
		} while (w >= 1 || w == 0);

		mult = sqrt((-2 * log(w)) / w);
		x1 = u1*mult;
		x2 = u2*mult;

		wind_horiz_sampled = wind_horiz_mean + wind_horiz_var*x1;
		wind_z_sampled = wind_vert_mean + wind_vert_var*x2;

		wind_horiz_direction = atan2(state_curr->get_vehicle()->getVehicleState().wind.wind_y, state_curr->get_vehicle()->getVehicleState().wind.wind_x);
		wind_x_sampled = wind_horiz_sampled*cos(wind_horiz_direction);
		wind_y_sampled = wind_horiz_sampled*sin(wind_horiz_direction);

		// dynamic equation

		airspeed_curr = _airspeed;																			// take perfect following in planning level prediction
		attitude_curr.pitch = atan2(g_param_curve[0]*pow(_airspeed, 2) + g_param_curve[1]*_airspeed + g_param_curve[2], _airspeed);	// function of L/D polar: given by equation
		attitude_curr.yaw = _heading*D2R;																	// take perfect following in planning level prediction

		state_curr->set_attitude(&attitude_curr);
		//state_curr->set_airspeed(&airspeed_curr);
		state_next->set_attitude(state_curr->get_attitude());
		//state_next->set_airspeed(state_curr->get_airspeed());

		ground_speed_x = airspeed_curr*cos(state_curr->get_attitude()->pitch)*cos(state_curr->get_attitude()->yaw) +wind_x_sampled;
		ground_speed_y = airspeed_curr*cos(state_curr->get_attitude()->pitch)*sin(state_curr->get_attitude()->yaw) +wind_y_sampled;
		ground_speed_z = airspeed_curr*sin(state_curr->get_attitude()->pitch)										 +wind_z_sampled;

		global_pos_next.lat = state_curr->get_global_pos()->lat + ground_speed_x*_duration / (EARTH_RADIUS*D2R);
		global_pos_next.lon = state_curr->get_global_pos()->lon + ground_speed_y*_duration / (EARTH_RADIUS*D2R);

		state_next->set_global_pos(&global_pos_next);

		state_next->get_terrain_map()->get_elevation(state_next->get_global_pos()->lat, state_next->get_global_pos()->lon, &elev);
		altitude_next.wrt_msl = state_curr->get_altitude()->wrt_msl + ground_speed_z*_duration;									
		altitude_next.wrt_terrain = state_curr->get_altitude()->wrt_msl + ground_speed_z*_duration - elev;

		state_next->set_altitude(&altitude_next);
		state_next->get_thermal_prediction_model()->get_thermal_prediction(global_pos_next.lat, global_pos_next.lon, &prob);

		yaw_curr = state_curr->get_vehicle()->getVehicleState().attitude.yaw;
		yaw_diff = _heading*D2R - yaw_curr;
		
		//heading angle wrap up
		while (abs(yaw_diff) > PI)
		{
			if (yaw_diff > PI)
			{
				yaw_diff = yaw_diff - 2 * PI;
			}	
			else if(yaw_diff < -PI)
			{
				yaw_diff = yaw_diff + 2 * PI;
			}
		}

		// REWARD FUNCTION:
		strength = 1; // unit: m/s
		
		*reward = pow(GAMMA, i_step)*(pow(_airspeed, 2) - (pow(state_curr->get_vehicle()->getVehicleState().vfrhud.true_airspeed, 2)) / 2
			+ GRAV_ACCELERATION*(state_next->get_altitude()->wrt_msl - state_curr->get_altitude()->wrt_msl)
			- abs(yaw_diff) / _duration*_airspeed*ALPHA
			+ GRAV_ACCELERATION*prob*strength/(g_param_inversion_ceiling - state_next->get_altitude()->wrt_msl))*_duration;

		return true;
	}

}


bool
Action::execute(const std::shared_ptr<MavLinkSailplane> vehicle) const
{
	// recompute ground speed using airspeed command
	float vx, vy, vz;
	float lat_cmd, lon_cmd;

	vx = _airspeed*cos(vehicle->getVehicleState().attitude.pitch)*cos(_heading*D2R) - vehicle->getVehicleState().wind.wind_x;
	vy = _airspeed*cos(vehicle->getVehicleState().attitude.pitch)*sin(_heading*D2R) - vehicle->getVehicleState().wind.wind_y;
	vz = _airspeed*sin(vehicle->getVehicleState().attitude.pitch)					- vehicle->getVehicleState().wind.wind_z;

	lat_cmd = vehicle->getVehicleState().global_est.pos.lat + vx *100 *_duration / (EARTH_RADIUS*D2R); // take 100 times further in order to make constant heading command
	lon_cmd = vehicle->getVehicleState().global_est.pos.lon + vy *100 *_duration / (EARTH_RADIUS*D2R); // take 100 times further in order to make constant heading command
	
	bool rc = false;

    vehicle->setTargetAirspeed(_airspeed);
    if (vehicle->getVehicleState().mode != static_cast<int>(APM_CUSTOM_FLIGHT_MODE::MANUAL))
    {
        return vehicle->moveToGlobalPosition(lat_cmd, lon_cmd, 100).wait(3000, &rc);
    }
    else
    {
        *g_out << Utils::to_string(Utils::now()) << "   ### Error: the navigation action was not attempted, because the plane is in the manual flight mode." << std::endl;
        return false;
    }
}


void
Action::print(std::ostream& out) const
{
	// print a description of this action to ostream
    out << "airspeed\t" << std::fixed << std::setprecision(5) << _airspeed << "\theading\t" << std::fixed << std::setprecision(5) << _heading << std::endl;
	//printf("aspd\t%3.2f\theading\t%3.2f\tduration\t%2.0d\tsoar\t", _airspeed, _heading, _duration);
}

