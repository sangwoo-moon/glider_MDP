#pragma once
#include <string>
#include <Map.hpp>

#define MIN_ELEVATION (-3000.0f)
#define MAX_ELEVATION (9000.0f)

class TerrainMap : private Map
{
public:

    using Map::initialize;

	// check whether elevation can be retrieved at a given latitude and longitude
	bool get_elevation(float latitude, float longitude, float *elev); 

	// check whether elevation can be retrieved at coordinates <x,y> (in meters) w.r.t. a point with given GPS coordinates
	bool get_elevation(float x, float y, float lat_home, float long_home, float *elev);
};