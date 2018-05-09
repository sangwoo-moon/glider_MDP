#pragma once
#include "CRoutePlanner.hpp"
#include "MavLinkConnection.hpp"
#include "MavLinkSailplane.hpp"
#include "ConsoleColor.h"
#include "Parameters.hpp"


using namespace mavlinkcom;
typedef mavlink_utils::Utils Utils;

CRoutePlanner::CRoutePlanner() :
	_mavlink_sailplane(nullptr),
	_terrain_map(nullptr),
	_thermal_prediction_model(nullptr),
	_geofence_map(nullptr),
	_actions_all(nullptr),
	_actions_glide(nullptr),
	_num_discr_heading_values(g_c_num_discr_heading_values_default),
	_num_discr_airspeed_values(g_c_num_discr_airspeed_values_default),
	_num_receding_horiz_values(g_param_receding_horiz_values),
	_reced_time(g_param_glide_action_duration),
	_planning_sleep(g_param_planning_sleep),
	_quit(false)
{
	State::initialize(_mavlink_sailplane, _terrain_map, _geofence_map, _thermal_prediction_model);
}


CRoutePlanner::CRoutePlanner(std::shared_ptr<MavLinkSailplane> mavlink_sailplane, std::shared_ptr<TerrainMap> terrain_map, 
	std::shared_ptr<Geofence> geofence_map, std::shared_ptr<ThermalPredictionModel> thermal_prediction_model,
	unsigned num_receding_horiz_values, unsigned reced_time, unsigned planning_sleep, unsigned num_discr_heading_values, unsigned num_discr_airspeed_values) :
	_mavlink_sailplane(mavlink_sailplane),
	_terrain_map(terrain_map),
	_geofence_map(geofence_map),
	_thermal_prediction_model(thermal_prediction_model),
	_num_receding_horiz_values(num_receding_horiz_values),
	_reced_time(reced_time),
	_planning_sleep(planning_sleep),
	_quit(false)
{
	State::initialize(_mavlink_sailplane, _terrain_map, _geofence_map, _thermal_prediction_model);
	create_actions(num_discr_heading_values, num_discr_airspeed_values);
}


CRoutePlanner::CRoutePlanner(std::shared_ptr<MavLinkSailplane> mavlink_sailplane, std::shared_ptr<TerrainMap> terrain_map,
	std::shared_ptr<Geofence> geofence_map, std::shared_ptr<ThermalPredictionModel> thermal_prediction_model,
	unsigned num_receding_horiz_values, unsigned reced_time, unsigned planning_sleep, float *discr_heading_values, unsigned num_discr_heading_values, float *discr_airspeed_values, unsigned num_discr_airspeed_values) :
	_mavlink_sailplane(mavlink_sailplane),
	_terrain_map(terrain_map),
	_geofence_map(geofence_map),
	_thermal_prediction_model(thermal_prediction_model),
	_num_receding_horiz_values(num_receding_horiz_values),
	_reced_time(reced_time),
	_planning_sleep(planning_sleep),
	_quit(false)
{
	State::initialize(_mavlink_sailplane, _terrain_map, _geofence_map, _thermal_prediction_model);
	create_actions(discr_heading_values, num_discr_heading_values, discr_airspeed_values, num_discr_airspeed_values);
    
    //TO DO: this can fail, would be good to register the return code somewhere or initialize automatically but outside the constructor
    initialize();
}


CRoutePlanner::~CRoutePlanner()
{
	cleanup_actions();
}

bool
CRoutePlanner::initialize()
{
    return _mavlink_sailplane->initializeForPlanning();
}

void 
CRoutePlanner::create_actions(unsigned num_discr_heading_values, unsigned num_discr_airspeed_values)
{
	cleanup_actions();

	_actions_all = new Action[2 + num_discr_heading_values * num_discr_airspeed_values];

	// Soar action. Note that the duration is set to 0 -- for the soar action this means that we don't know a-priori how long it will last
	_actions_all[0] = Action(0, 0, true, 0);

	// Stop-soaring action.
	_actions_all[1] = Action(0, 0, false, 0);

	_actions_glide = new Action*[num_discr_heading_values];

	unsigned actions_all_idx = 2;
	float heading_step = 360.0f / num_discr_heading_values;
	float airspeed_step = (g_c_airspeed_max - g_c_airspeed_min) / num_discr_airspeed_values + 1; // +1 is for including the endpoints of the airspeed range

	for (unsigned i = 0; i < num_discr_heading_values; i++, actions_all_idx++)
	{
		_actions_glide[i] = new Action[num_discr_airspeed_values];

		for (unsigned j = 0; j < num_discr_airspeed_values; j++)
		{
			_actions_glide[i][j] = Action(i * heading_step, j * airspeed_step, false, _reced_time);
			_actions_all[actions_all_idx] = _actions_glide[i][j];
			actions_all_idx++;
		}
	}

	_num_discr_heading_values = num_discr_heading_values;
	_num_discr_airspeed_values = num_discr_airspeed_values;
}


void 
CRoutePlanner::create_actions(float *discr_heading_values, unsigned num_discr_heading_values, float *discr_airspeed_values, unsigned num_discr_airspeed_values)
{
	cleanup_actions();

	_actions_all = new Action[2 + num_discr_heading_values * num_discr_airspeed_values];

	// Soar action. Note that the duration is set to 0 -- for the soar action this means that we don't know a-priori how long it will last
	_actions_all[0] = Action(0, 0, true, 0);

	// Stop-soaring action.
	_actions_all[1] = Action(0, 0, false, 0);

	_actions_glide = new Action*[num_discr_heading_values];

	unsigned actions_all_idx = 2;

	for (unsigned i = 0; i < num_discr_heading_values; i++) // actions_all_idx++)
	{
		_actions_glide[i] = new Action[num_discr_airspeed_values];

		for (unsigned j = 0; j < num_discr_airspeed_values; j++)
		{
			_actions_glide[i][j] = Action(discr_heading_values[i], discr_airspeed_values[j], false, _reced_time);
			_actions_all[actions_all_idx] = _actions_glide[i][j];
			actions_all_idx++;
		}
	}

	_num_discr_heading_values = num_discr_heading_values;
	_num_discr_airspeed_values = num_discr_airspeed_values;
}


void 
CRoutePlanner::cleanup_actions()
{
	if (_actions_all != nullptr)
	{
		delete[] _actions_all;
	}

	if (_actions_glide != nullptr)
	{
		for (unsigned i = 0; i < _num_discr_heading_values; i++)
		{
			if (_actions_glide[i])
			{
				delete[] _actions_glide[i];
			}
		}

		delete[] _actions_glide;
	}
}


// UCT algorithm (bool is for checking whether this algorithm is feasible because of geofence)
bool
CRoutePlanner::UCT(State *state_curr, const std::shared_ptr<MavLinkSailplane> vehicle, unsigned *execution_idx)
{

	unsigned *opt_action_idx = new unsigned[_num_receding_horiz_values];					// create indices of action set
	unsigned *opt_action_idx_temp = new unsigned[_num_receding_horiz_values];				// create indices of action set (temp, for single branch)
	unsigned  n_s;																			// create n_s
	unsigned *n_sa = new unsigned[_num_discr_heading_values*_num_discr_airspeed_values];	// create n_sa
	float	 *Q_hat = new float[_num_discr_heading_values*_num_discr_airspeed_values];	// create approximated reward array

	bool flag_visit = false;
	float total_cost = -1e10; // total cost for comparison

	float *cum_reward = new float[_num_receding_horiz_values + 1]; // consider terminating NULL (+1)
	unsigned i_max_step = 0;
	State state_pair[2];										// create current and next state structure (for single pair)

	unsigned i_UCT = 0;
	while (i_UCT < ITER)
	{

		state_pair[0].copy_state(state_curr);
		cum_reward[0] = 0;

		// TO DO:: MAKE UCT SOLVER
		for (unsigned i_step = 0; i_step < _num_receding_horiz_values; i_step++)
		{
			// initialization for current state (w.r.t first visit)
			if (!flag_visit)
			{
				n_s = 1; // ad-hoc to prevent from ln(0) = -inf
				for (unsigned i_act = 0; i_act < _num_discr_heading_values*_num_discr_airspeed_values; i_act++)
				{
					n_sa[i_act] = 1; // ad-hoc to prevent from 0 denominator
					Q_hat[i_act] = 0;
				}
				flag_visit = true;
			}

			i_max_step = i_step;

			// figure out best action using roll out procedure
			float reward_comp = -1e10;
			for (unsigned i_act = 2; i_act < 2 + _num_discr_heading_values*_num_discr_airspeed_values; i_act++) // without soaring mode
			{
				float reward;
				double compensator;
				_actions_all[i_act].sample_transition(&state_pair[0], &state_pair[1], &reward, i_step);
				compensator = (double)sqrt(log(n_s) / n_sa[i_act]);

				if (reward_comp < (reward + C_UCT* compensator))
				{
					reward_comp = reward;
					opt_action_idx_temp[i_step] = i_act;
				}
			}

			// execute best action in current state
			float reward;
			_actions_all[opt_action_idx_temp[i_step]].sample_transition(&state_pair[0], &state_pair[1], &reward, i_step);
			cum_reward[i_step + 1] = cum_reward[i_step] + reward;

			// proceed to next state given best action sequence
			state_pair[0].copy_state(&state_pair[1]);

		}

		if (cum_reward[_num_receding_horiz_values - 1] > total_cost)
		{
			total_cost = cum_reward[_num_receding_horiz_values - 1];
			for (unsigned i_step = 0; i_step < i_max_step + 1; i_step++)
				opt_action_idx[i_step] = opt_action_idx_temp[i_step];
		}

		i_UCT++;
	}

	delete[] cum_reward;
		
	delete[] n_sa;
	delete[] Q_hat;
	delete[] opt_action_idx_temp;

	*execution_idx = opt_action_idx[0];

	// for checking whether the result from planner cannot updated because of the boundary
	if (!_actions_all[opt_action_idx[0]].is_feasible_in_state(state_curr))
	{
		return false;
	}
	
	return true;

}


void
CRoutePlanner::run()
{
	// A placeholder for logic that will ask the planning algorithm to produce actions, execute those actions, monitor action execution, etc.
	
	/* PLANNER-SOARING CONTROLLER REVISED LOGIC */
	//
	// if (glider is inside geofence)
	//     - take UCT
	//	   if (the UCT can make reasonable result)
	//		    for (i=1;i< {planning_sleep}/{sec}; i++)
	//              if (soaring controller takes detection/soaring)
	//                  - engage soaring command
	//              else
	//                  - inhibit soaring command
	//                  - take planning guidance
	//              end
	//              - take one sec pause
	//          end
	//     else
	//          if (soaring controller takes detection/soaring)
	//              - engage soaring command
	//          else
	//              - inhibit soaring command
	//              - nav_to_home
	//          end
	//          - take one sec pause
	//     end
	// else
	//     - inhibit soaring command
	//     - nav_to_home
	//     - take one sec pause
	// end

	bool rc = false;
	State state_curr;

	float lat_home = g_home[0];
	float lon_home = g_home[1]; 

	while (!time_to_quit())
	{


		// 1. Take current state
		state_curr.get_curr_state();


		// 2. check the geofence based on current state
		if ((state_curr.get_geofence_map()->is_allowed(state_curr.get_global_pos()->lat, state_curr.get_global_pos()->lon, state_curr.get_altitude()->wrt_msl)))
		{
			// 2-1. run the planner and see if the planner makes admissible result
			if (CRoutePlanner::UCT(&state_curr, _mavlink_sailplane, &_idx_action_execution))
			{
				for (unsigned i = 0; i < g_param_planning_sleep; i++)
				{

					// display soaring status
					if (_mavlink_sailplane->getVehicleState().soar_state.soaring == 0)
					{
						*g_out << Utils::to_string(Utils::now()) << "   soaring: 0" << std::endl;
					}
					else if (_mavlink_sailplane->getVehicleState().soar_state.soaring == 1)
					{
						*g_out << Utils::to_string(Utils::now()) << "   soaring: 1" << std::endl;
					}
					else
					{
						*g_out << Utils::to_string(Utils::now()) << "   soaring: 2" << std::endl;
					}

					// 2-2. check whether the soaring controller gets soaring
					if (_mavlink_sailplane->getVehicleState().soar_state.soaring != 0)
					{

						// engage soaring command
						rc = false;
						_mavlink_sailplane->setTargetAirspeed(g_param_aspd_soar);

						if (!_mavlink_sailplane->inhibitSoaring(0, *g_out).wait(g_param_ack_wait_timeout, &rc) || !rc)
						{
							*g_out << Utils::to_string(Utils::now()) << red << "   ### Error: inhib_soar(0) failed." << white << std::endl << std::endl;
						}
						else
						{
							*g_out << Utils::to_string(Utils::now()) << "   inhib_soar: 0\tMSL\t" << _mavlink_sailplane->getVehicleState().vfrhud.altitude << "\taspd:\t" << g_param_aspd_soar << std::endl << std::endl;
						}

					}
					else
					{

						// inhibit soaring command
						rc = false;
						if (!_mavlink_sailplane->inhibitSoaring(1, *g_out).wait(g_param_ack_wait_timeout, &rc) || !rc)
						{
							*g_out << Utils::to_string(Utils::now()) << red << "   ### Error: inhib_soar(1) failed." << white << std::endl;
						}
						else
						{
							*g_out << Utils::to_string(Utils::now()) << "   inhib_soar: 1" << std::endl << std::endl;
						}

						// take planning action
						_actions_all[_idx_action_execution].execute(_mavlink_sailplane);
						print(*g_out);

					}

					// planning pause for 1 second
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}

			}
			else
			{

				// display soaring status
				if (_mavlink_sailplane->getVehicleState().soar_state.soaring == 0)
				{
					*g_out << Utils::to_string(Utils::now()) << "   soaring: 0" << std::endl;
				}
				else if (_mavlink_sailplane->getVehicleState().soar_state.soaring == 1)
				{
					*g_out << Utils::to_string(Utils::now()) << "   soaring: 1" << std::endl;
				}
				else
				{
					*g_out << Utils::to_string(Utils::now()) << "   soaring: 2" << std::endl;
				}

				if (_mavlink_sailplane->getVehicleState().soar_state.soaring != 0)
				{
					// engage soaring command
					rc = false;
					_mavlink_sailplane->setTargetAirspeed(g_param_aspd_soar);

					if (!_mavlink_sailplane->inhibitSoaring(0, *g_out).wait(g_param_ack_wait_timeout, &rc) || !rc)
					{
						*g_out << Utils::to_string(Utils::now()) << red << "   ### Error: inhib_soar(0) failed." << white << std::endl << std::endl;
					}
					else
					{
						*g_out << Utils::to_string(Utils::now()) << "   inhib_soar: 0\tMSL\t" << _mavlink_sailplane->getVehicleState().vfrhud.altitude << "\taspd:\t" << g_param_aspd_soar << std::endl << std::endl;
					}

				}
				else
				{
					// inhibit soaring command
					rc = false;
					if (!_mavlink_sailplane->inhibitSoaring(1, *g_out).wait(g_param_ack_wait_timeout, &rc) || !rc)
					{
						*g_out << Utils::to_string(Utils::now()) << red << "   ### Error: inhib_soar(1) failed." << white << std::endl;
					}
					else
					{
						*g_out << Utils::to_string(Utils::now()) << "   inhib_soar: 1" << std::endl << std::endl;
					}

					// nav_to_home
					*g_out << Utils::to_string(Utils::now()) << yellow << "   ### Warning: close to geofence." << white << std::endl;
					rc = false;
					_mavlink_sailplane->moveToGlobalPosition(lat_home, lon_home, 100).wait(g_param_ack_wait_timeout, &rc);;
					*g_out << Utils::to_string(Utils::now()) << "   Nav to home." << white << std::endl << std::endl;
					*g_out << std::endl;
				}

				// homing wait pause
				std::this_thread::sleep_for(std::chrono::seconds(homing_wait_sec));
			}

		}
		else
		{

			// display soaring status
			if (_mavlink_sailplane->getVehicleState().soar_state.soaring == 0)
			{
				*g_out << Utils::to_string(Utils::now()) << "   soaring: 0" << std::endl;
			}
			else if (_mavlink_sailplane->getVehicleState().soar_state.soaring == 1)
			{
				*g_out << Utils::to_string(Utils::now()) << "   soaring: 1" << std::endl;
			}
			else
			{
				*g_out << Utils::to_string(Utils::now()) << "   soaring: 2" << std::endl;
			}

			// inhibit soaring command
			rc = false;
			if (!_mavlink_sailplane->inhibitSoaring(1, *g_out).wait(g_param_ack_wait_timeout, &rc) || !rc)
			{
				*g_out << Utils::to_string(Utils::now()) << red << "   ### Error: inhib_soar(1) failed." << white << std::endl;
			}
			else
			{
				*g_out << Utils::to_string(Utils::now()) << "   inhib_soar: 1" << std::endl << std::endl;
			}

			// nav_to_home
			*g_out << Utils::to_string(Utils::now()) << yellow << "   ### Warning: outside of geofence." << white << std::endl;
			rc = false;
			_mavlink_sailplane->moveToGlobalPosition(lat_home, lon_home, 100).wait(g_param_ack_wait_timeout, &rc);;
			*g_out << Utils::to_string(Utils::now()) << "   Nav to home." << white << std::endl << std::endl;
			*g_out << std::endl;

			// homing wait pause
			std::this_thread::sleep_for(std::chrono::seconds(homing_wait_sec));
		}

	}

}



void
CRoutePlanner::quit()
{	
	// put the sailplane into the FBWA mode before finishing the planner, so that a human can safely take over
    *g_out << Utils::to_string(Utils::now()) << "   Putting the sailplane into the FBWA mode." << white << std::endl;
    _mavlink_sailplane->setStabilizedFlightMode();

	std::lock_guard<std::mutex> lock(_quit_mutex);
	_quit = true;
}


bool
CRoutePlanner::time_to_quit()
{
	std::lock_guard<std::mutex> lock(_quit_mutex);
	return _quit;
}


void 
CRoutePlanner::print(std::ostream& out)
{
	// To be filled in
/*    State state_curr;
    state_curr.get_curr_state();
    out << green << "Current state: " << white;
    state_curr.print(out);*/
	out << Utils::to_string(Utils::now()) << yellow << "   planning: " << white;
	_actions_all[_idx_action_execution].print(out);
    out << "\n";
}
