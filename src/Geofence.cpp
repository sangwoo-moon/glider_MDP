#include "Geofence.hpp"
#include "ConsoleColor.h"
#include "Parameters.hpp"
#include <filesystem>
#include <iostream>

typedef mavlink_utils::Utils Utils;

bool 
Geofence::initialize(std::string allowed_territories_dir)
{
    // list all files in the specified directory, pick out those with the ".csv" extension, and try to load a terrain map
    // from each of them
    std::experimental::filesystem::directory_iterator dir_end;
    std::string extension(".csv");
    
    try
    {
        unsigned file_counter = 0;
         
        for (std::experimental::filesystem::directory_iterator it(allowed_territories_dir); it != dir_end; it++)
        {
            if (std::experimental::filesystem::is_regular_file(it->path()))
            {
                std::string file_path = it->path().string();

                if (file_path.rfind(extension) == file_path.length() - extension.length())
                {
                    file_counter++;
                    TerrainMap *allowed_territory = new TerrainMap();
                    allowed_territory->initialize(file_path, MIN_ELEVATION, MAX_ELEVATION);
                    _allowed_territories.push_back(allowed_territory);
                }
            }
        }

        if (file_counter == 0)
        {
            std::cout << red << "Error: directory \"" << allowed_territories_dir << " doesn't contain any csv files." << white << std::endl;
            return false;
        }
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return false;
    }

    return true;
}


bool
Geofence::is_allowed(double latitude, double longitude, float altitude_msl_meters)
{
    // Check every loaded map. The point being checked is considered within the geofence if all of the following
    // conditions are met:
    //
    //  - its lat/lon are on one of the loaded maps
    //  - its alt is more than the height of the terrain/obstacle + some g_param_alt_slack at that lat/lon
    for (unsigned int i = 0; i < _allowed_territories.size(); i++)
    {
        float alt = MIN_ELEVATION;

        if (_allowed_territories[i]->get_elevation(latitude, longitude, &alt) && 
			(alt + g_param_alt_slack < altitude_msl_meters) && (MIN_ELEVATION + g_param_alt_slack < altitude_msl_meters) && 
			(altitude_msl_meters < alt + g_param_max_alt) && (altitude_msl_meters < MAX_ELEVATION))
        {
            return true;
        }

    }

    return false;
}


bool 
Geofence::is_allowed(float x, float y, double lat_home, double long_home, float altitude_msl_meters)
{
    // Check every loaded map. The point being checked is considered within the geofence if all of the following
    // conditions are met:
    //
    //  - its lat/lon are on one of the loaded maps
    //  - its alt is more than the height of the terrain/obstacle + some g_param_alt_slack at that lat/lon
    for (unsigned int i = 0; i < _allowed_territories.size(); i++)
    {
        float alt = MIN_ELEVATION;

        if (_allowed_territories[i]->get_elevation(x, y, lat_home, long_home, &alt) && (alt + g_param_alt_slack < altitude_msl_meters) && (MIN_ELEVATION + g_param_alt_slack < altitude_msl_meters) && (altitude_msl_meters < alt + g_param_max_alt) && (altitude_msl_meters < MAX_ELEVATION))
        {
            return true;
        }
    }

    return false;
}