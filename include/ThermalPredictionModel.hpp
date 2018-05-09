#pragma once
#include <string>
#include <Map.hpp>


class ThermalPredictionModel
{
private:

    Map _thermal_prob_map;

public:

    bool initialize(std::string thermal_prediction_map);
    // get a thermal prediction for a given latitude and longitude
    bool get_thermal_prediction(float latitude, float longitude, float alt, float *prob, float *rad, float *strength);
    // get a thermal prediction for coordinates <x,y,alt> (in meters) w.r.t. a point with given GPS coordinates
    bool get_thermal_prediction(float x, float y, float alt, float lat_home, float long_home, float *prob, float *rad, float *strength);
	bool get_thermal_prediction(float latitude, float longitude, float *prob);
};