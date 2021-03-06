#pragma once

#include <string>
#include <vector>
#include <functional>

#include "connection_result.h"

namespace dronecore {

class DroneCoreImpl;
class Device;

/**
 * @brief This is the main class of **%DroneCore MAVLink API Library** (for the Dronecode Platform).

 * It is used to discover vehicles and manage active connections.
 *
 * An instance of this class must be created (first) in order to use the library.
 * The instance must be destroyed after use in order to break connections and release all resources.
 */
class DroneCore
{
public:
    static constexpr auto DEFAULT_UDP_CONNECTION_URL = "udp://:14540";
    static constexpr int DEFAULT_UDP_PORT = 14540;
    static constexpr auto DEFAULT_TCP_REMOTE_IP = "127.0.0.1";
    static constexpr int DEFAULT_TCP_REMOTE_PORT = 5760;
    static constexpr auto DEFAULT_SERIAL_DEV_PATH = "/dev/ttyS0";
    static constexpr int DEFAULT_SERIAL_BAUDRATE = 57600;

    /**
     * @brief Constructor.
     */
    DroneCore();

    /**
     * @brief Destructor.
     *
     * Disconnects all connected vehicles and releases all resources.
     */
    ~DroneCore();

    /**
     * @brief Adds Connection via URL
     *
     * Supports connection: Serial, TCP or UDP.
     * Connection URL format should be:
     * - UDP - udp://[Bind_host][:Bind_port]
     * - TCP - tcp://[Server_host][:Server_port]
     * - Serial - serial://[Dev_Node][:Baudrate]
     *
     * Default URL : udp://:14540.
     * - Default Bind host IP is localhost(127.0.0.1)
     *
     * @param connection_url connection URL string.
     * @return The result of adding the connection.
     */
    ConnectionResult add_any_connection(const std::string &connection_url = DEFAULT_UDP_CONNECTION_URL);

    /**
     * @brief Adds a UDP connection to the specified port number.
     *
     * @param local_port_number The local UDP port to listen to (defaults to 14540, the same as mavros).
     * @return The result of adding the connection.
     */
    ConnectionResult add_udp_connection(int local_port_number = DEFAULT_UDP_PORT);

    /**
     * @brief Adds a TCP connection with a specific IP address and port number.
     *
     * @param remote_ip Remote IP address to connect to (defaults to 127.0.0.1).
     * @param remote_port The TCP port to connect to (defaults to 5760).
     * @return The result of adding the connection.
     */
    ConnectionResult add_tcp_connection(const std::string &remote_ip = DEFAULT_TCP_REMOTE_IP,
                                        int remote_port = DEFAULT_TCP_REMOTE_PORT);

    /**
     * @brief Adds a serial connection with a specific port (COM or UART dev node) and baudrate as specified.
     *
     * @param dev_path COM or UART dev node name/path (defaults to "/dev/ttyS0").
     * @param baudrate Baudrate of the serial port (defaults to 57600).
     * @return The result of adding the connection.
     */
    ConnectionResult add_serial_connection(const std::string &dev_path = DEFAULT_SERIAL_DEV_PATH,
                                           int baudrate = DEFAULT_SERIAL_BAUDRATE);

    /**
     * @brief Get vector of device UUIDs.
     *
     * This returns a vector of the UUIDs of all devices that have been discovered.
     * If a device doesn't have a UUID then DroneCore will instead use its MAVLink system ID (range: 0..255).
     *
     * @return A reference to the vector containing the UUIDs.
     */
    const std::vector<uint64_t> &device_uuids() const;

    /**
     * @brief Get the first discovered device.
     *
     * This returns the first discovered device or a null device if no device has yet been found.
     *
     * @return A reference to a device.
     */
    Device &device() const;

    /**
     * @brief Get the device with the specified UUID.
     *
     * This returns a device for a given UUID if such a device has been discovered and a null
     * device otherwise.
     *
     * @param uuid UUID of device to get.
     * @return A reference to the specified device.
     */
    Device &device(uint64_t uuid) const;

    /**
     * @brief Callback type for discover and timeout notifications.
     *
     * @param uuid UUID of device (or MAVLink system ID for devices that don't have a UUID).
     */
    typedef std::function<void(uint64_t uuid)> event_callback_t;

    /**
     * @brief Returns `true` if exactly one device is currently connected.
     *
     * Connected means we are receiving heartbeats from this device.
     * It means the same as "discovered" and "not timed out".
     *
     * If multiple devices have connected, this will return `false`.
     *
     * @return `true` if exactly one device is connected.
     */
    bool is_connected() const;

    /**
     * @brief Returns `true` if a device is currently connected.
     *
     * Connected means we are receiving heartbeats from this device.
     * It means the same as "discovered" and "not timed out".
     *
     * @param uuid UUID of device to check.
     * @return `true` if device is connected.
     */
    bool is_connected(uint64_t uuid) const;

    /**
     * @brief Register callback for device discovery.
     *
     * This sets a callback that will be notified if a new device is discovered.
     *
     * **Note** Only one callback can be registered at a time. If this function is called several
     * times, previous callbacks will be overwritten.
     *
     * @param callback Callback to register.
     *
     */
    void register_on_discover(event_callback_t callback);

    /**
     * @brief Register callback for device timeout.
     *
     * This sets a callback that will be notified if no heartbeat of the device has been received
     * in 3 seconds.
     *
     * **Note** Only one callback can be registered at a time. If this function is called several
     * times, previous callbacks will be overwritten.
     *
     * @param callback Callback to register.
     */
    void register_on_timeout(event_callback_t callback);

private:
    /* @private. */
    DroneCoreImpl *_impl;

    // Non-copyable
    DroneCore(const DroneCore &) = delete;
    const DroneCore &operator=(const DroneCore &) = delete;

    /* Adds a connection for Network protocol*/
    ConnectionResult add_link_connection(const std::string &protocol, const std::string &ip,
                                         const int port);
};

} // namespace dronecore
