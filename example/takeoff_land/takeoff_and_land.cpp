//
// Simple example to demonstrate how to use DroneCore.
//
// Author: Julian Oes <julian@oes.ch>

#include <chrono>
#include <cstdint>
#include <dronecore/action.h>
#include <dronecore/dronecore.h>
#include <dronecore/telemetry.h>
#include <iostream>
#include <thread>

using namespace dronecore;
using namespace std::this_thread;
using namespace std::chrono;

#define ERROR_CONSOLE_TEXT "\033[31m" //Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" //Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m"  //Restore normal console colour

void usage(std::string arg);

int main(int argc, char **argv)
{
    DroneCore dc;
    std::string connection_url;
    ConnectionResult connection_result;

    bool discovered_device = false;
    if (argc == 1) {
        usage(argv[0]);
        connection_result = dc.add_any_connection();
    } else {
        connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    }

    if (connection_result != ConnectionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: "
                  << connection_result_str(connection_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    std::cout << "Waiting to discover device..." << std::endl;
    dc.register_on_discover([&discovered_device](uint64_t uuid) {
        std::cout << "Discovered device with UUID: " << uuid << std::endl;
        discovered_device = true;
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a device after around 2 seconds.
    sleep_for(seconds(2));

    if (!discovered_device) {
        std::cout << ERROR_CONSOLE_TEXT << "No device found, exiting." << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // We don't need to specify the UUID if it's only one device anyway.
    // If there were multiple, we could specify it with:
    // dc.device(uint64_t uuid);
    Device &device = dc.device();

    auto telemetry = std::make_shared<Telemetry>(&device);
    auto action = std::make_shared<Action>(&device);

    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Setting rate failed:" << Telemetry::result_str(
                      set_rate_result) << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }


    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry->position_async([](Telemetry::Position position) {
        std::cout << TELEMETRY_CONSOLE_TEXT // set to blue
                  << "Altitude: " << position.relative_altitude_m << " m"
                  << NORMAL_CONSOLE_TEXT // set to default color again
                  << std::endl;
    });


    // Check if vehicle is ready to arm
    while (telemetry->health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        sleep_for(seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action->arm();

    if (arm_result != Action::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << Action::result_str(
                      arm_result) << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Take off
    std::cout << "Taking off..." << std::endl;
    const Action::Result takeoff_result = action->takeoff();
    if (takeoff_result != Action::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Takeoff failed:" << Action::result_str(
                      takeoff_result) << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Let it hover for a bit before landing again.
    sleep_for(seconds(10));

    std::cout << "Landing..." << std::endl;
    const Action::Result land_result = action->land();
    if (land_result != Action::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Land failed:" << Action::result_str(
                      land_result) << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(5));
    std::cout << "Finished..." << std::endl;
    return 0;
}

void usage(std::string arg)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << arg << " [connection_url]" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl;
    std::cout << "Default connection URL is udp://:14540" << std::endl;
}
