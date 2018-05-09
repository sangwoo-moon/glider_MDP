// Copyright (c) Microsoft Corporation. All rights reserved.


#include "Utils.hpp"
#include "FileSystem.hpp"
#include "MavLinkConnection.hpp"
#include "MavLinkSailplane.hpp"
#include "MavLinkMessages.hpp"
#include "MavLinkLog.hpp"
#include "Semaphore.hpp"
#include "CRoutePlanner.hpp"
#include "State.hpp"
#include "Action.hpp"
#include "TerrainMap.hpp"
#include "ThermalPredictionModel.hpp"
#include "Geofence.hpp"
#include "ConsoleColor.h"
#include "Parameters.hpp"
#include <iostream>
#include <vector>
#include <string.h>
#include <functional>
#include <mutex>
#include <map>
#include <ctime>


#if defined(_WIN32)
#include <filesystem>
// for some unknown reason, VC++ doesn't define this handy macro...
#define __cpp_lib_experimental_filesystem 201406
#else
#if __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
#endif
#endif

using namespace mavlinkcom;
//using namespace mavlinkcom_impl;

typedef mavlink_utils::Utils Utils;
typedef mavlink_utils::FileSystem FileSystem;
typedef unsigned int uint;

class PortAddress
{
public:
	std::string addr;
	int port;
};

// This is used for connecting to the sailplane's Pixhawk via a serial port (using a telemetry radio or a cable)
bool g_main_connection_exists = false;
std::string g_com_port = "";
int g_baud_rate = 57600;

// This is used for connecting to X-Plane via TCP or UDP
PortAddress TcpUdpEndPoint;
#define DEFAULT_MAIN_CONNECTION_TCP_PORT 5760
#define DEFAULT_MAIN_CONNECTION_UDP_PORT 14560
std::string defaultLocalAddress{ "127.0.0.1" };

bool g_quit = false;

// The connection to the sailplane
std::shared_ptr<MavLinkConnection> g_main_connection;

// These ports are used to echo the mavlink messages to other apps such as Mission Planner.
std::vector<PortAddress> g_proxyEndPoints;
uint g_prevLastProxyEndPoint = 0;
std::vector<PortAddress> usedPorts;
#define DEFAULT_PROXY_UDP_PORT 14580

CRoutePlanner* g_planner = nullptr;
// The main worker thread
std::thread g_planning_thread;

void PrintUsage() {
	std::cout << green << "USAGE: RoutePlanner.exe serial<params>|tcp<params>|udp<params> [other options]" << white << std::endl;
	printf("Connects to PX4 or SITL over TCP, UDP, or serial COM ports, runs the planner, and proxies\n");
	printf("between the PX4 and other MAVLink nodes such as GCSs.\n");
	std::cout << yellow << "Options: " << white << std::endl;
	printf("    serial[:comPortName][,baudrate]]      - connect to a sailplane at a given serial port\n");
	printf("    tcp[:ipAddress][,baudrate]]           - connect to a given address and port via TCP\n");
	printf("    udp[:ipAddress][,baudrate]]           - connect to a given address and port via UDP\n");
	printf("    proxy:ipaddr[:port]                   - send all mavlink messages to and from remote node (via UDP)\n");
	printf("                                            You can specify \"proxy\" multiple times with different\n");
	printf("                                            port numbers to proxy drone messages out to multiple listeners\n");
	printf("    local:ipaddr                          - use the local machine's network IP address specified by ipaddr\n");
	printf("                                            when acting as a proxy. By default, it's 127.0.0.1.\n");
    printf("    run_planner                           - start the route planner.\n");
    printf("    init_planner                          - initilize the route planner. Use this command if the planner's\n");
	printf("    fin_planner                           - finish the route planner.\n");
    printf("                                            default initialization attempt failed and it could not get\n");
    printf("                                            parameters from the sailplane.\n");
    printf("    nav_to:<lat>,<lon>                    - tell the sailplane to navigate to a point with the given\n");
    printf("                                            latitude and longitude. Make sure there are no spaces in this command.\n");
    printf("                                            Example: nav_to:47.687789,-121.943700\n");
    printf("    set_airspeed:<arspd>                  - set airspeed to <arspd> (in m/s)\n");
    printf("    inhib_soaring:<0|1>                   - stop the sailplane from soaring (1) or tell it it's OK to soar (0)\n");
    printf("                                            when it finds a good thermal.\n");
    printf("                                            Example: inhib_soaring:true\n");
    printf("    mode                                  - show the current flight mode\n");
	printf("    home                                  - show current home location based on LLH\n");
	printf("    tas                                   - show current TAS from vfrhud of Mavlink_sailplane\n");
	printf("    gs                                    - show current GS from vfrhud of Mavlink_sailplane\n");
	printf("    wind                                  - show current wind velocity with respect to inertia frame from Mavlink_sailplane\n");
    printf("    quit                                  - stop the planner and exit the program\n\n\n");
	std::cout << yellow << "EXAMPLE. " << white;
	printf("The following sequence of commands connects to a PixHawk on serial port COM5 via a telemetry\n");
	printf("radio link, launching the planner, and then in addition starts forwarding MAVLink messages to and from\n");
	printf("two ground control stations via UDP, a local one and a remote one:\n\n");

	std::cout << green << "RoutePlanner.exe serial:COM5,57600" << white << std::endl;
	std::cout << green << ">proxy:192.168.1.5,14560" << white << std::endl;
	std::cout << green << ">local:192.168.1.5" << white << std::endl;
	std::cout << green << ">proxy:10.81.228.50,14580" << white << std::endl;
	printf("\n");
    //nav_to:47.687789,-121.943699
}


std::shared_ptr<MavLinkConnection> connectSerial()
{
	printf("Connecting to sailplane at serial port %s at baud rate %d\n", g_com_port.c_str(), g_baud_rate);
	std::shared_ptr<MavLinkConnection> serialConnection = MavLinkConnection::connectSerial("sailplane", g_com_port, g_baud_rate);
	printf("Connection established.\n");

	return serialConnection;
}


std::shared_ptr<MavLinkConnection> connectProxy(const PortAddress& endPoint, std::string name)
{
	printf("Connecting to UDP proxy address %s:%d\n", endPoint.addr.c_str(), endPoint.port);
	std::shared_ptr<MavLinkConnection> proxyConnection = MavLinkConnection::connectRemoteUdp(name, defaultLocalAddress, endPoint.addr, endPoint.port);
	// forward all PX4 messages to the remote proxy and all messages from remote proxy to PX4.
	g_main_connection->join(proxyConnection);
	printf("Connection established.\n");

	return proxyConnection;
}


std::shared_ptr<MavLinkConnection> connectTcp()
{
	if (TcpUdpEndPoint.addr == "") {
		TcpUdpEndPoint.addr = defaultLocalAddress;
	}

	printf("Connecting to SITL at address %s:%d\n", TcpUdpEndPoint.addr.c_str(), TcpUdpEndPoint.port);
	std::shared_ptr<MavLinkConnection> tcpConnection = MavLinkConnection::connectTcp("sailplane", defaultLocalAddress, TcpUdpEndPoint.addr, TcpUdpEndPoint.port);
	printf("Connection established.\n");

	return tcpConnection;
}


std::shared_ptr<MavLinkConnection> connectUdp()
{
	if (TcpUdpEndPoint.addr == "") {
		TcpUdpEndPoint.addr = defaultLocalAddress;
	}

	printf("Connecting to address %s:%d\n", TcpUdpEndPoint.addr.c_str(), TcpUdpEndPoint.port);
	std::shared_ptr<MavLinkConnection> udpConnection = MavLinkConnection::connectRemoteUdp("sailplane", defaultLocalAddress, TcpUdpEndPoint.addr, TcpUdpEndPoint.port);
	printf("Connection established.\n");

	return udpConnection;
}


bool HandleCommandLine(std::vector<std::string> cmdline_args)
{
	// parse command line
	for (uint i = 0; i < cmdline_args.size(); i++)
	{
		const char* arg = cmdline_args[i].c_str();

		std::string option(arg);
		std::vector<std::string> parts = Utils::split(option, ":,", 2);
		std::string lower = Utils::toLower(parts[0]);

		if (lower == "serial")
		{
			if (g_main_connection_exists)
			{
				std::cout << red << "Error: a serial connection with a vehicle already exists. Can't start another vehicle connection" << white << std::endl;
				// This is a non-fatal error
				return true;
			}

			if (parts.size() > 1)
			{
				g_com_port = parts[1];
				if (parts.size() > 2)
				{
					g_baud_rate = atoi(parts[2].c_str());
					if (g_baud_rate == 0)
					{
						std::cout << red << " ### Error: invalid baud rate in \"serial\" argument" << white << std::endl;
						return false;
					}
				}

				try
				{
					// The connection will get reestablished if the link breaks, as long as the other endpoint doesn't restart in the meantime.
					g_main_connection = connectSerial();
					g_main_connection_exists = true;
				}
				catch (std::runtime_error e)
				{
					std::cout << e.what() << std::endl;
					return false;
				}
			}
		}
		else if (lower == "udp")
		{
			TcpUdpEndPoint.port = DEFAULT_MAIN_CONNECTION_UDP_PORT;
			if (parts.size() > 1)
			{
				TcpUdpEndPoint.addr = parts[1];
				if (parts.size() > 2)
				{
					TcpUdpEndPoint.port = atoi(parts[2].c_str());
				}

				try
				{
					// The connection will get reestablished if the link breaks, as long as the other endpoint doesn't restart in the meantime.
					g_main_connection = connectUdp();
					g_main_connection_exists = true;
				}
				catch (std::runtime_error e)
				{
					std::cout << e.what() << std::endl;
					return false;
				}
			}
		}
		else if (lower == "tcp")
		{
			TcpUdpEndPoint.port = DEFAULT_MAIN_CONNECTION_TCP_PORT;
			if (parts.size() > 1)
			{
				TcpUdpEndPoint.addr = parts[1];
				if (parts.size() > 2)
				{
					TcpUdpEndPoint.port = atoi(parts[2].c_str());
				}

				try
				{
					g_main_connection = connectTcp();
					g_main_connection_exists = true;
				}
				catch (std::runtime_error e)
				{
					std::cout << e.what() << std::endl;
					return false;
				}
			}
		}
		else if (lower == "local")
		{
			if (parts.size() > 1)
			{
				defaultLocalAddress = parts[1];
			}
		}
		else if (lower == "proxy")
		{
			if (!g_main_connection_exists)
			{
                std::cout << red << "### Error: no connection to the sailplane -- create it first. Please type in \"RoutePlanner.exe -?\" for usage instructions." << white << std::endl; //" ### Error: a port for the main sailplane connection has not been specified or is not the first argument in the list." << white << std::endl;
				return false;
			}

			PortAddress ep;
			ep.port = DEFAULT_PROXY_UDP_PORT;

			if (parts.size() > 1)
			{
				ep.addr = parts[1];
				if (parts.size() > 2)
				{
					ep.port = atoi(parts[2].c_str());
				}
			}
			g_proxyEndPoints.push_back(ep);
		}
        else if (lower == "test")
        {
            // put whatever you want to test here
        }
		else if (lower == "-h" || lower == "-?" || lower == "-help" || lower == "--help")
		{
			PrintUsage();
			return false;
		}
        else if (lower == "nav_to")
        {
            if (g_planner == nullptr)
            {
                std::cout << "### Error: planner not initialized yet." << std::endl;
                return false;
            }

            if (parts.size() == 3)
            {
                try
                {
                    float lat = std::stof(parts[1], nullptr);
                    float lon = std::stof(parts[2], nullptr);
                    std::cout << "Vectoring the sailplane to the point with latitude " << std::setprecision(9)<< lat << " and longitutude " << lon << std::endl;
                    
                    bool rc = false;

                        if (!g_planner->get_sailplane()->moveToGlobalPosition(lat, lon, 100, std::cout).wait(g_param_ack_wait_timeout, &rc) || !rc)
                        {
                            std::cout << "Navigation failed" << std::endl;
                        }
                        else
                        {
                            std::cout << "Navigation in progress" << std::endl;
                        }
                }
                catch (std::invalid_argument)
                {
                    std::cout << red << "### Error: invalid value for one of the command parameters. " << white << std::endl;
                    return false;
                }
                catch (std::out_of_range)
                {
                    std::cout << red << "### Error: out-of-range value for one of the command parameters. " << white << std::endl;
                    return false;
                }
                catch (std::exception &e)
                {
                    std::cout << red << e.what() << white << std::endl;
                    return false;
                }
            }
            else
            {
                std::cout << "### Error: the command needs to have two parameters, latitude and longitude, but " << parts.size() - 1 << " parameters were specified." << std::endl;
            }
        }

		else if (lower == "set_home")
		{
			if (g_planner == nullptr)
			{
				std::cout << "### Error: planner not initialized yet." << std::endl;
				return false;
			}

			if (parts.size() == 3)
			{
				try
				{
					float lat = std::stof(parts[1], nullptr);
					float lon = std::stof(parts[2], nullptr);
					std::cout << "Setting home for the sailplane to the point with latitude " << std::setprecision(9) << lat << " and longitutude " << lon << std::endl;

					g_home[0] = lat;
					g_home[1] = lon;

					std::cout << "Home position is re-set." << std::endl;
				}
				catch (std::invalid_argument)
				{
					std::cout << red << "### Error: invalid value for one of the command parameters. " << white << std::endl;
					return false;
				}
				catch (std::out_of_range)
				{
					std::cout << red << "### Error: out-of-range value for one of the command parameters. " << white << std::endl;
					return false;
				}
				catch (std::exception &e)
				{
					std::cout << red << e.what() << white << std::endl;
					return false;
				}
			}
			else
			{
				std::cout << "### Error: the command needs to have two parameters, latitude and longitude, but " << parts.size() - 1 << " parameters were specified." << std::endl;
			}
		}

        else if (lower == "set_airspeed")
        {
            if (g_planner == nullptr)
            {
                std::cout << "### Error: planner not initialized yet." << std::endl;
                return false;
            }

            if (parts.size() == 2)
            {
                try
                {
                    float arspd = std::stof(parts[1], nullptr);
                    std::cout << "Setting target aispeed to " << arspd << " m/s" << std::endl;

                    if (!g_planner->get_sailplane()->setTargetAirspeed(arspd, std::cout))
                    {

                        std::cout << "Airspeed could not be set" << std::endl;
                    }
                    else
                    {
                        std::cout << "Airspeed set" << std::endl;
                    }
                }
                catch (std::invalid_argument)
                {
                    std::cout << red << "### Error: invalid value for one of the command parameters. " << white << std::endl;
                    return false;
                }
                catch (std::out_of_range)
                {
                    std::cout << red << "### Error: out-of-range value for one of the command parameters. " << white << std::endl;
                    return false;
                }
                catch (std::exception &e)
                {
                    std::cout << red << e.what() << white << std::endl;
                    return false;
                }
            }
            else
            {
                std::cout << red << "### Error: the command needs to have two parameters, latitude and longitude, but " << parts.size() - 1 << " parameters were specified." << white << std::endl;
            }
        }
        else if (lower == "inhib_soaring")
        {
            if (parts.size() == 2)
            {
                bool rc = false;
                if (parts[1] == "1")
                {
                    if (!g_planner->get_sailplane()->inhibitSoaring(1, std::cout).wait(g_param_ack_wait_timeout, &rc) || !rc)
                    {
                        std::cout << red << "### Error: command failed." << white << std::endl;
                    }
                    else
                    {
                        std::cout << "Success!" << std::endl;
                    }
                }
                else if (parts[1] == "0")
                {
                    if (!g_planner->get_sailplane()->inhibitSoaring(0, std::cout).wait(g_param_ack_wait_timeout, &rc) || !rc)
                    {
                        std::cout << red << "### Error: command failed." << white << std::endl;
                    }
                    else
                    {
                        std::cout << "Success!!" << std::endl;
                    }
                }
                else
                {
                    std::cout << red << "### Error: invalid argument. Should be \"1\" or \"0\"." << white << std::endl;
                }
            }
            else
            {
                std::cout << red << "### Error: this command doesn't take any arguments." << white << std::endl;
            }

            return false;
        }
        else if (lower == "init_planner")
        {
            if (parts.size() == 1)
            {
                std::cout << yellow << "Initializing the planner..." << white << std::endl;
				if (!g_planner->initialize())
				{
					std::cout << "### Error: could not get all the parameters from the sailplane. Initialization failed." << std::endl;
				}
            }
            else
            {
                std::cout << red << "### Error: this command doesn't take any arguments." << white << std::endl;
            }

            return false;
        }
        else if (lower == "run_planner")
        {
            if (parts.size() != 1)
            {
                std::cout << red << "### Error: invalid number of arguments to this command." << white << std::endl;
            }

            if (!g_main_connection_exists)
            {
                std::cout << red << " ### Error: connection to the sailplane hasn't been created." << white << std::endl;
                std::cout << red << "Please type in \"RoutePlanner.exe -?\" for usage instructions." << white << std::endl;
                std::cout << yellow << "Quitting..." << white << std::endl;
                return 1;
            }
            // The vehicle starts sending the hearbeats
            std::shared_ptr<MavLinkSailplane> mavlink_sailplane = std::make_shared<MavLinkSailplane>(72, 1);
            mavlink_sailplane->connect(g_main_connection);
            mavlink_sailplane->startHeartbeat();

            /* Terrain map loading */
			std::shared_ptr<TerrainMap> terrain_map = std::make_shared<TerrainMap>();
            std::cout << green << "Terrain map file reading started." << white << std::endl;
            terrain_map->initialize(g_param_terrain_map_file, MIN_ELEVATION, MAX_ELEVATION);
            
			/* Thermal map loading */
			std::shared_ptr<ThermalPredictionModel> thermal_prediction_model = std::make_shared<ThermalPredictionModel>();
            std::cout << green << "Thermal model file reading started." << white << std::endl;
            thermal_prediction_model->initialize(g_param_thermal_map_file);
            
			/* Geofence loading */
            std::shared_ptr<Geofence> geofence = std::make_shared<Geofence>();
            std::cout << green << "Geofence files reading started." << white << std::endl;
            geofence->initialize(g_param_allowed_territories_dir);

            float headings[] = { 0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345 };

            g_planner = new CRoutePlanner(mavlink_sailplane, terrain_map, geofence, thermal_prediction_model,
                g_param_receding_horiz_values, g_param_glide_action_duration, g_param_planning_sleep, headings, sizeof(headings) / sizeof(headings[0]), g_param_airspeeds, sizeof(g_param_airspeeds) / sizeof(g_param_airspeeds[0]));


			if (g_planner == nullptr)
			{
				std::cout << red << "### Error: planner could not be created." << white << std::endl;
				return false;
			}
            else
            {
                std::cout << yellow << "Initializing the planner..." << white << std::endl;
			
                if (!g_planner->initialize())
                {
                    std::cout << "### Error: could not get all the parameters from the sailplane. Initialization failed." << std::endl;
                    return false;
                }
            }

            std::cout << yellow << "Starting the planner..." << white << std::endl;
            g_planning_thread = std::thread(&CRoutePlanner::run, g_planner);
            
            return false;
        }
		else if (lower == "fin_planner")
		{
			if (g_planner == nullptr)
			{
				std::cout << red << "### Error: planner not initialized yet." << white << std::endl;
				return false;
			}
			if (parts.size() == 1)
			{
				std::cout << yellow << "Finishing the planner..." << white << std::endl;
				g_planner->quit();
				std::cout << ">";
			}
			else
			{
				std::cout << red << "### Error: invalid number of arguments to this command." << white << std::endl;
			}
			return false;
		}
		else if (lower == "quit")
		{
            if (parts.size() == 1)
            {
                g_quit = true;
                std::cout << yellow << "Quitting..." << white << std::endl;

                if (g_planner != nullptr)
                {
                    g_planner->quit();
                }
                
                if (g_planning_thread.joinable())
                {
                    g_planning_thread.join();
                }

                return true;
            }
            else
            {
                std::cout << red << "### Error: invalid number of arguments to this command." << white << std::endl;
                return false;
            }
		}
        else if (lower == "mode")
        {
            std::cout << green << "Current flight mode: " << white << g_planner->get_sailplane()->getVehicleState().mode << green << "; armed status: " << white << g_planner->get_sailplane()->getVehicleState().controls.armed << std::endl;
        }
		else if (lower == "home") // show home location
		{
			std::cout << green << "Home Location | " << white << "Lat: " << std::fixed << std::setprecision(6) << g_home[0] << "\tLon: " << std::fixed << std::setprecision(6) << g_home[1] << std::endl;

			return false;
		}
		else if (lower == "tas") // show true air speed
		{
			std::cout << green << "True Airspeed | " << white << g_planner->get_sailplane()->getVehicleState().vfrhud.true_airspeed << std::endl;

			return false;
		}
		else if (lower == "wind") // show wind profile
		{
			std::cout << green << "Wind Velocity | " << white << "X: " << g_planner->get_sailplane()->getVehicleState().wind.wind_x << "\tY: " 
				<< g_planner->get_sailplane()->getVehicleState().wind.wind_y << "\tZ: " << g_planner->get_sailplane()->getVehicleState().wind.wind_z << std::endl;

			return false;
		}
		else if (lower == "gs") // show ground speed
		{
			std::cout << green << "Ground Speed | " << white << g_planner->get_sailplane()->getVehicleState().vfrhud.groundspeed << std::endl;

			return false;
		}
		else
		{
            std::cout << red << "### Error: unexpected argument: " << white;
			printf("%s\n\n", arg);
			PrintUsage();
			return false;
		}

		for (uint i = g_prevLastProxyEndPoint; i < g_proxyEndPoints.size(); i++)//(auto ptr = g_proxyEndPoints.begin(), end = g_proxyEndPoints.end(); ptr != end; ptr++)
		{
			bool usablePort = true;
			PortAddress proxyEndPoint = g_proxyEndPoints[i];
			for (auto ep = usedPorts.begin(), endep = usedPorts.end(); ep != endep; ep++)
			{
				PortAddress used = *ep;
				if (used.addr == proxyEndPoint.addr && used.port == proxyEndPoint.port)
				{
					printf("Cannot proxy to address that is already used: %s:%d\n", used.addr.c_str(), used.port);
					usablePort = false;
					break;
				}
			}

			if (usablePort)
			{
				try
				{
					connectProxy(proxyEndPoint, "GCS_" + proxyEndPoint.addr + "_" + std::to_string(proxyEndPoint.port));
					usedPorts.push_back(proxyEndPoint);
				}
				catch (std::runtime_error e)
				{
					std::cout << e.what() << std::endl;
				}
			}

			g_prevLastProxyEndPoint++;
		}
	}

	return true;
}


int main(int argc, const char* argv[])
{
	std::vector<std::string> cmdline_args;

	for (int i = 1; i < argc; i++)
	{
		cmdline_args.push_back(std::string(argv[i]));
	}

	while (!HandleCommandLine(cmdline_args))
	{
		std::string cmdline_input;
		std::cout << ">";
		std::cin >> cmdline_input;
		cmdline_args.clear();
		cmdline_args = Utils::split(cmdline_input, " ", 1);
	}

    std::ostringstream oss;
    oss << "planner_log_" << Utils::to_string(Utils::now()) << ".txt";
    std::ofstream out_fs;
    out_fs.open(oss.str().c_str(), std::ios::out);
    g_out = &out_fs;

	while (!g_quit)
	{
		std::string cmdline_input;
		std::cout << ">";
		std::cin >> cmdline_input;
		cmdline_args.clear();
		cmdline_args = Utils::split(cmdline_input, " ", 1);
        HandleCommandLine(cmdline_args);
	}

    out_fs.close();
	
	/*
	std::this_thread::sleep_for(std::chrono::seconds(5));

	VehicleState state = mavlink_sailplane->getVehicleState();
	printf("Home position is %s, %f,%f,%f\n", state.home.is_set ? "set" : "not set",
		state.home.global_pos.lat, state.home.global_pos.lon, state.home.global_pos.alt);

	bool rc = false;
	if (!mavlink_sailplane->armDisarm(true).wait(g_param_ack_wait_timeout, &rc) || !rc) {
		printf("arm command failed\n");
		return 1;
	}
	else
	{
		printf("vehicle armed\n");
	}

	rc = false;

	if (!mavlink_sailplane->armDisarm(false).wait(g_param_ack_wait_timeout, &rc) || !rc) {
		printf("disarm command failed\n");
		return 1;
	}
	else
	{
		printf("vehicle disarmed\n");
	}
	*/
	return 0;
}