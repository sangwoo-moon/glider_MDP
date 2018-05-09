#include "ThermalPredictionModel.hpp"
#include "ConsoleColor.h"
#include <iostream>

#define MIN_PROB (0.0f)
#define MAX_PROB (1.0f)

bool 
ThermalPredictionModel::initialize(std::string thermal_prediction_map)
{
    return _thermal_prob_map.initialize(thermal_prediction_map, MIN_PROB, MAX_PROB);
}

bool 
ThermalPredictionModel::get_thermal_prediction(float latitude, float longitude, float alt, float *prob, float *rad, float *strength)
{
    if (prob == nullptr )
    {
        std::cout << red << "Error: ThermalPredictionModel::get_thermal_prediction: the prob pointer is null" << white << std::endl;
        return false;
    }
    if (rad == nullptr)
    {
        std::cout << red << "Error: ThermalPredictionModel::get_thermal_prediction: the radius pointer is null" << white << std::endl;
        return false;
    }
    if (strength == nullptr)
    {
        std::cout << red << "Error: ThermalPredictionModel::get_thermal_prediction: the strength pointer is null" << white << std::endl;
        return false;
    }

    bool valid = _thermal_prob_map.get_value(latitude, longitude, prob);

    // TO DO: do something more intelligent instead of the following; use actual models for retrieving the prediction components (hard-coded now)
    *strength = 1.0f;
    *rad = 50.0f;
    *prob = 0.8f;

    return valid;
}

bool 
ThermalPredictionModel::get_thermal_prediction(float x, float y, float alt, float lat_home, float long_home, float *prob, float *rad, float *strength)
{
    if (prob == nullptr)
    {
        std::cout << red << "Error: ThermalPredictionModel::get_thermal_prediction: the prob pointer is null" << white << std::endl;
        return false;
    }
    if (rad == nullptr)
    {
        std::cout << red << "Error: ThermalPredictionModel::get_thermal_prediction: the radius pointer is null" << white << std::endl;
        return false;
    }
    if (strength == nullptr)
    {
        std::cout << red << "Error: ThermalPredictionModel::get_thermal_prediction: the strength pointer is null" << white << std::endl;
        return false;
    }

    bool valid = _thermal_prob_map.get_value(x, y, lat_home, long_home, prob);

    // TO DO: do something more intelligent instead of the following; use actual models for retrieving the prediction components (hard-coded now)
    *strength = 1.0f;
    *rad = 50.0f;
    *prob = 0.8f;

    return valid;
}

bool
ThermalPredictionModel::get_thermal_prediction(float latitude, float longitude, float *prob)
{
	if (prob == nullptr)
	{
		std::cout << red << "Error: ThermalPredictionModel::get_thermal_prediction: the prob pointer is null" << white << std::endl;
		return false;
	}

	bool valid = _thermal_prob_map.get_value(latitude, longitude, prob);

	return valid;
}