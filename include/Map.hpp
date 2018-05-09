#pragma once
#include <string>



#define EARTH_RADIUS (6378137.0f)
#define D2R (0.0174533f)
#define PI (3.141592f)

struct MapSize {
    unsigned num_cols; // Number of cells on the map from west to east
    unsigned num_rows; // Number of cells on the map from north to south
};

struct MapResolution {
    float horiz; // horizontal resolution in m
    float vert;  // vertical resolution in m
};

struct MapBound {
    // boundary lattitude
    float north_lat;
    float south_lat;

    // boundary longitude
    float east_lon;
    float west_lon;

    // boundary in local coordinates with respect to home position
    // coordinate is NED (North-x, East-y)
    float x_min;
    float y_min;

    float x_max;
    float y_max;
};


class Map
{
private:

    // The map is assumed to be a rectangular grid consisting of square cells with M-meter sides. The map assigns a value
    // to each cell.

    // WARNING: this implementation assumes that the piece of Earth's surface covered by the given map is small enough to be considered rectangular 

    // Maps cell indexes <x, y> w.r.t. the top north-western corner of the map to elevations
    float **_map_xy2value;

    MapSize _map_size;
    MapResolution _map_resolution;
    MapBound _map_bound;

public:

    ~Map();

    // Reads the file, assuming it contains numeric data in the .csv format, and verifies values
    // in it against the specified limits
    bool initialize(const std::string map_file, float min_limit, float max_limit);

    // retrieves the value at a given latitude and longitude
    bool get_value(float latitude, float longitude, float *value);

    // retrieves the value at coordinates <x,y> (in meters) w.r.t. a point with given GPS coordinates
    bool get_value(float x, float y, float lat_home, float long_home, float *value);

	// Get distance (in meters) between two points represented as LLH cooridnates.
	float get_lat_distance(float lat_1, float lat_2);
	float get_lon_distance(float lon_1, float lon_2, float lat_1, float lat_2);
};
