#pragma once

#include "State.hpp"
#include "Parameters.hpp"
#include <random>

#define GRAV_ACCELERATION (9.802f)
#define ENERGY_CEILING (1600.0f)

// Airspeed for the optimal glide ratio, in m/s
const float OPT_GLIDE_AIRSPEED = 12.5; // from field-test

// Default action duration in seconds (defult initializaiton)
const unsigned DEFAULT_DURATION = 10;

const float GAMMA = 0.9f;
const float ALPHA = 0.9f; // thermal seeking oriented when approaches to 1
const float ASPD_PARAM = 15.0f; // for airspeed related reward parameter
const float TIME_MARGIN = 2.0f; // time margin for geo-fence violation


class Action
{
private:

	float _heading;
	float _airspeed;
	bool _soar;
	unsigned _duration;



public:

	Action();
	Action(float heading, float airspeed, bool soar, unsigned duration);
	~Action();

	bool is_feasible_in_state(State *state) const;
	bool sample_transition(State *state_curr, State *state_next, float *reward, unsigned i_step) const;
	bool execute(const std::shared_ptr<MavLinkSailplane> vehicle) const;
	void print(std::ostream& out) const;

};