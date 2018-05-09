#include "Map.hpp"
#include "Utils.hpp"
#include "ConsoleColor.h"
#include "Parameters.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <vector>


typedef mavlink_utils::Utils Utils;

#define NUM_ROWS_FIELD_IDX 3
#define NUM_COLS_FIELD_IDX 5
#define METERS_PER_PIXEL_FIELD_IDX 7
#define NORTH_LAT_BOUNDARY_FIELD_IDX 9
#define SOUTH_LAT_BOUNDARY_FIELD_IDX 11
#define EAST_LON_BOUNDARY_FIELD_IDX 13
#define WEST_LON_BOUNDARY_FIELD_IDX 15

#define MAX_NUM_COLS 50000
#define MAX_NUM_ROWS 50000
#define MAX_NUM_METERS_PER_PIXEL 60
#define MIN_LONGITUDE (-180.0f)
#define MAX_LONGITUDE (180.0f)
#define MIN_LATITUDE (-90.0f)
#define MAX_LATITUDE (90.0f)

bool
Map::initialize(const std::string map_file, float min_limit, float max_limit)
{
    bool has_invalid_values = false;
    // import all value info into Map class
    // Example file name: imgn39w119_13_rows_10812_cols_10812_mpp_10_nlat_039.00055_slat_037.99944_elon_-117.99944_wlon_-119.00055.csv
    size_t last_slash = map_file.rfind('\\');
    if (last_slash == std::string::npos)
    {
        last_slash = map_file.rfind('/');
    }

    if (last_slash == std::string::npos)
    {
        last_slash = -1;
    }

    std::string map_file_name = map_file.substr(last_slash + 1, map_file.length() - last_slash - 1);
    std::cout << map_file_name << std::endl;
    std::vector<std::string> file_name_parts = Utils::split(map_file_name, "_", 1);

    try
    {
        // Parse the number of columns from the map file name
        if (NUM_COLS_FIELD_IDX >= file_name_parts.size())
        {
            std::cout << red << "Error: map file name doesn't contain the number of columns" << white << std::endl;
            return false;
        }

        int num_cols = std::stoi(file_name_parts[NUM_COLS_FIELD_IDX], nullptr, 10);

        if (num_cols > 0 && num_cols < MAX_NUM_COLS)
        {
            _map_size.num_cols = (unsigned int)num_cols;
        }
        else
        {
            std::cout << red << "Error: invalid number of columns in the map file name: " << num_cols << white << std::endl;
            return false;
        }

        // Parse the number of rows from the map file name
        if (NUM_ROWS_FIELD_IDX >= file_name_parts.size())
        {
            std::cout << red << "Error: map file name doesn't contain the number of rows" << white << std::endl;
            return false;
        }

        int num_rows = std::stoi(file_name_parts[NUM_ROWS_FIELD_IDX], nullptr, 10);

        if (num_rows > 0 && num_rows < MAX_NUM_ROWS)
        {
            _map_size.num_rows = (unsigned int) num_rows;
        }
        else
        {
            std::cout << red << "Error: invalid number of rows in the map file name: " << num_rows << white << std::endl;
            return false;
        }

        // Parse the map resolution, in meters per pixel, from the map file name
        if (METERS_PER_PIXEL_FIELD_IDX >= file_name_parts.size())
        {
            std::cout << red << "Error: map file name doesn't contain the number of meters per pixel (resolution)" << white << std::endl;
            return false;
        }

        float mmp = std::stof(file_name_parts[METERS_PER_PIXEL_FIELD_IDX], nullptr);

        if (mmp > 0 && mmp < MAX_NUM_METERS_PER_PIXEL)
        {
            _map_resolution.horiz = mmp;
            _map_resolution.vert = mmp;
        }
        else
        {
            std::cout << red << "Error: invalid number of meters per pixel in the map file name: " << mmp << white << std::endl;
            return false;
        }

        // Parse the latitude of the map's northern boundary from the map file name
        if (NORTH_LAT_BOUNDARY_FIELD_IDX >= file_name_parts.size())
        {
            std::cout << red << "Error: map file name doesn't contain the latitude of the map's northern boundary" << white << std::endl;
            return false;
        }

        float nl = std::stof(file_name_parts[NORTH_LAT_BOUNDARY_FIELD_IDX], nullptr);

        if (nl >= MIN_LATITUDE && nl <= MAX_LATITUDE)
        {
            _map_bound.north_lat = nl;
        }
        else
        {
            std::cout << red << "Error: invalid latitude of the map's northern boundary in the map file name: " << nl << white << std::endl;
            return false;
        }

        // Parse the latitude of the map's southern boundary from the map file name
        if (SOUTH_LAT_BOUNDARY_FIELD_IDX >= file_name_parts.size())
        {
            std::cout << red << "Error: map file name doesn't contain the latitude of the map's southern boundary" << white << std::endl;
            return false;
        }

        float sl = std::stof(file_name_parts[SOUTH_LAT_BOUNDARY_FIELD_IDX], nullptr);

        if (sl >= MIN_LATITUDE && sl <= MAX_LATITUDE)
        {
            _map_bound.south_lat = sl;
        }
        else
        {
            std::cout << red << "Error: invalid  latitude of the map's southern boundary in the map file name: " << sl << white << std::endl;
            return false;
        }

        // Parse the latitude of the map's eastern boundary from the map file name
        if (EAST_LON_BOUNDARY_FIELD_IDX >= file_name_parts.size())
        {
            std::cout << red << "Error: map file name doesn't contain the latitude of the map's eastern boundary" << white << std::endl;
            return false;
        }

        float el = std::stof(file_name_parts[EAST_LON_BOUNDARY_FIELD_IDX], nullptr);

        if (el >= MIN_LONGITUDE && el <= MAX_LONGITUDE)
        {
            _map_bound.east_lon = el;
        }
        else
        {
            std::cout << red << "Error: invalid longitude of the map's eastern boundary in the map file name: " << el << white << std::endl;
            return false;
        }

        // Parse the latitude of the map's western boundary from the map file name
        if (WEST_LON_BOUNDARY_FIELD_IDX >= file_name_parts.size())
        {
            std::cout << red << "Error: map file name doesn't contain the latitude of the map's western boundary" << white << std::endl;
            return false;
        }

        float wl = std::stof(file_name_parts[WEST_LON_BOUNDARY_FIELD_IDX], nullptr);

        if (wl >= MIN_LONGITUDE && wl <= MAX_LONGITUDE)
        {
            _map_bound.west_lon = wl;
        }
        else
        {
            std::cout << red << "Error: invalid longitude of the map's western boundary in the map file name: " << wl << white << std::endl;
            return false;
        }
    }
    catch (std::invalid_argument)
    {
        std::cout << red << "Error: invalid value for one of the fields in the map file name. " << white << std::endl;
        return false;
    }
    catch (std::out_of_range)
    {
        std::cout << red << "Error: out-of-range value for one of the fields in the map file name. " << white << std::endl;
        return false;
        // if the connum_rowsed value would fall out of the range of the result type 
        // or if the underlying function (std::strtol or std::strtoull) sets errno 
        // to ERANGE.
    }
    catch (...)
    {
        std::cout << red << "Error when parsing map file name \"" << map_file << "\"" << white << std::endl;
        return false;
    }

    // initialize 2d point-array for value storing. Note that overall sizes are given.
    _map_xy2value = new float*[_map_size.num_rows];

    std::ifstream file(map_file);

    if (!file)
    {
        std::cout << red << "Error: file " << map_file << " doesn't exist" << white << std::endl;
        return false;
    }
    
	unsigned step = 0; // step for displaying trigger of map file loading

    for (unsigned i_num_rows = 0; i_num_rows < _map_size.num_rows; i_num_rows++)
    {
        _map_xy2value[i_num_rows] = new float[_map_size.num_cols];
        std::string line;
        std::getline(file, line);

        if (!file.good())
        {
            std::cout << red << "Error: parsing the map file had to be terminated before all columns could be filled. Row during termination: " << i_num_rows << white << std::endl;
            std::vector<std::string> s = Utils::split(line, ",", 1);
            std::cout << "Num tokens " << s.size() << std::endl;
            return false;
        }
		
        std::stringstream iss(line);
		
        for (unsigned i_num_cols = 0; i_num_cols < _map_size.num_cols; i_num_cols++)
        {
            std::string val;
			std::getline(iss, val, ','); // check whether the value is parsed by toggle (or ,)

            if (!iss.good() && (i_num_cols < _map_size.num_cols - 1))
            {
                std::cout << red << "Error: parsing the map file had to be terminated before all columns of row " << i_num_rows << " could be filled. Column during termination: " << i_num_cols << white << std::endl;
                return false;
            }

            //std::stringstream connum_rowsor(val);
            //connum_rowsor >> _map_xy2value[i_num_rows][i_num_cols];
			
            try
            {
				_map_xy2value[i_num_rows][i_num_cols] = std::stof(val, nullptr);

                if (_map_xy2value[i_num_rows][i_num_cols] < min_limit || _map_xy2value[i_num_rows][i_num_cols] > max_limit)
                {
                    has_invalid_values = true;
                }
            }
            catch (...)
            {
//                std::cout << red << "Error: invalid entry in row " << i_num_rows << ", column " << i_num_cols << " of the map file: " << val << white << std::endl;
                return false;
            }
        }
/*
		if ( step < 25 && (i_num_rows >= (int)(floor(_map_size.num_rows / 25))) && (i_num_rows % (int)(floor(_map_size.num_rows/25.0f)) == 0) )
		{
			step++;
			std::cout << "Map Reading   ";
			for (unsigned i = 0; i < step; i++)
			{
				std::cout << "*";
			}
			for (int i = 0; i < 4 - step; i++)
			{
				std::cout << " ";
			}
			std::cout << "| " << (int)(step * 25) << " % progressed" << std::endl;
		}
*/
    }

    return !has_invalid_values;
}


Map::~Map()
{
    // Release any memory allocated to the private member variables
    if (_map_xy2value != nullptr)
    {
        for (unsigned i = 0; i < _map_size.num_rows; i++)
        {
            if (_map_xy2value[i])
            {
                delete[] _map_xy2value[i];
            }
        }

        delete[] _map_xy2value;
    }
}


bool
Map::get_value(float latitude, float longitude, float *elev)
{

	float *dist_num_cols = (float *)malloc(sizeof(float));
	float *dist_num_rows = (float *)malloc(sizeof(float));
	unsigned *idx_num_cols = (unsigned *)malloc(sizeof(unsigned));
	unsigned *idx_num_rows = (unsigned *)malloc(sizeof(unsigned));

    // Returns true iff the method managed to successfully retrieve the value for the specified point
	if ( (longitude > (_map_bound.east_lon - g_param_map_buffer/(D2R*EARTH_RADIUS)) ) || (longitude < (_map_bound.west_lon + g_param_map_buffer / (D2R*EARTH_RADIUS)) ) )
	{
		return false;
	}
	else
	{
		if ( (latitude < (_map_bound.south_lat + g_param_map_buffer / (D2R*EARTH_RADIUS)) ) || (latitude > (_map_bound.north_lat - g_param_map_buffer / (D2R*EARTH_RADIUS)) ) )
		{
			return false;
		}
	}

    //returns value w.r.t. llh location (from north-west boundary)
    *dist_num_cols = min(_map_resolution.horiz*_map_size.num_cols-1,max(0,get_lon_distance(_map_bound.west_lon, longitude, _map_bound.north_lat, latitude)));	// positive (from West to East)
    *dist_num_rows = min(_map_resolution.vert*_map_size.num_rows-1,max(0,get_lat_distance(_map_bound.north_lat, latitude)));										// positive (from North to South)

    *idx_num_cols = int(floor(*dist_num_cols / _map_resolution.horiz) + 1);
    *idx_num_rows = int(floor(*dist_num_rows / _map_resolution.vert) + 1);

    *elev = _map_xy2value[*idx_num_rows - 1][*idx_num_cols - 1];

	free(dist_num_cols);
	free(dist_num_rows);
	free(idx_num_cols);
	free(idx_num_rows);

    return true;
}


bool
Map::get_value(float x, float y, float lat_home, float long_home, float *elev)
{
    // Returns true iff the method managed to successfully retrieve the value for the specified point

    _map_bound.y_min = get_lon_distance(long_home, _map_bound.west_lon, lat_home, _map_bound.north_lat);
    _map_bound.y_max = get_lon_distance(long_home, _map_bound.east_lon, lat_home, _map_bound.north_lat);

    _map_bound.x_min = get_lat_distance(lat_home, _map_bound.south_lat);
    _map_bound.x_max = get_lat_distance(lat_home, _map_bound.north_lat);

	if ((x < _map_bound.x_min) || (x > _map_bound.x_max))
	{
		return false;
	}
	else
	{
		if ((y < _map_bound.y_min) || (y > _map_bound.y_max))
		{
			return false;
		}
	}
        
    unsigned idx_num_cols = int(floor(y - _map_bound.y_min) / _map_resolution.horiz);
    unsigned idx_num_rows = int(floor(_map_bound.x_max - x) / _map_resolution.vert);

    *elev = _map_xy2value[idx_num_rows - 1][idx_num_cols - 1];
    return true;
}


/*
Presuming a spherical Earth with radius R (see below), and that the
locations of the two points in spherical coordinates (longitude and
latitude) are lon1,lat1 and lon2,lat2, then the Haversine Formula
(from R. W. Sinnott, "Virtues of the Haversine," Sky and Telescope,
vol. 68, no. 2, 1984, p. 159):

dlon = lon2 - lon1
dlat = lat2 - lat1
a = (sin(dlat/2))^2 + cos(lat1) * cos(lat2) * (sin(dlon/2))^2
c = 2 * atan2(sqrt(a), sqrt(1-a))
d = R * c

will give mathematically and computationally exact results. The
intermediate result c is the great circle distance in radians. The
great circle distance d will be in the same units as R.
*/

float
Map::get_lat_distance(float lat_1, float lat_2)
{
    float dlat = (lat_2 - lat_1)*D2R;
    float a = pow(sin(dlat / 2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float dist_lat = EARTH_RADIUS*c;

    return dist_lat;
}

float
Map::get_lon_distance(float lon_1, float lon_2, float lat_1, float lat_2)
{
    float dlon = (lon_2 - lon_1)*D2R;
    float a = cos(lat_1*D2R)*cos(lat_2*D2R)*pow(sin(dlon / 2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float dist_lon = EARTH_RADIUS*c;

    return dist_lon;
}