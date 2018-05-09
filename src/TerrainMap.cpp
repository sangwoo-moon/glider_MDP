#include "TerrainMap.hpp"



bool 
TerrainMap::get_elevation(float latitude, float longitude, float *elev)
{ 
    return get_value(latitude, longitude, elev);
}


bool 
TerrainMap::get_elevation(float x, float y, float lat_home, float long_home, float *elev)
{
	return get_value(x,y,lat_home, long_home, elev);
}