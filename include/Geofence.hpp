#pragma once
#include <string>
#include <vector>
#include <TerrainMap.hpp>
#include "FileSystem.hpp"



/*
The Geofence class allows checking whether a point in 3D space is within a geofenced territory and above any obstacles
at its GPS location.
*/


class Geofence
{
private:

    // A list of rectangular maps s.t. any GPS location in any of these maps is within the geofence. Thus,
    // we can geofence any territory representable as a set of (possibly overlapping) rectangles. The fewer rectangles
    // in the representation, the more efficient the geofence checks.
    std::vector<TerrainMap *> _allowed_territories;

public:

    bool initialize(std::string allowed_territories_dir);
    bool is_allowed(double latitude, double longitude, float altitude_msl_meters);
    // performs a geofence check at coordinates <x, y>(in meters) w.r.t.a point with given GPS coordinates
    bool is_allowed(float x, float y, double lat_home, double long_home, float altitude_msl_meters);
};