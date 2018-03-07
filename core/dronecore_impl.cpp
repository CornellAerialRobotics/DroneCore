#include "dronecore_impl.h"

#include <mutex>

#include "connection.h"
#include "global_include.h"
#include "tcp_connection.h"
#include "udp_connection.h"

#ifndef WINDOWS
#include "serial_connection.h"
#endif

namespace dronecore {

DroneCoreImpl::DroneCoreImpl() :
    _connections_mutex(),
    _connections(),
    _devices_mutex(),
    _devices(),
    _on_discover_callback(nullptr),
    _on_timeout_callback(nullptr)
{}

DroneCoreImpl::~DroneCoreImpl()
{
    {
        std::lock_guard<std::recursive_mutex> lock(_devices_mutex);
        _should_exit = true;

        for (auto system_it = _devices.begin(); system_it != _devices.end(); ++system_it) {
            for (auto component : system_it->second) {
                delete component;
            }
        }
        _devices.clear();
    }

    std::vector<Connection *> tmp_connections;
    {
        std::lock_guard<std::mutex> lock(_connections_mutex);

        // We need to copy the connections to a temporary vector. This way they won't
        // get used anymore while they are cleaned up.
        tmp_connections = _connections;
        _connections.clear();
    }

    for (auto connection : tmp_connections) {
        delete connection;
    }
}

void DroneCoreImpl::receive_message(const mavlink_message_t &message)
{
    // Don't ever create a device with sysid 0.
    if (message.sysid == 0) {
        return;
    }

    // FIXME: Ignore messages from QGroundControl for now. Usually QGC identifies
    //        itself with sysid 255.
    //        A better way would probably be to parse the heartbeat message and
    //        look at type and check if it is MAV_TYPE_GCS.
    if (message.sysid == 255) {
        return;
    }

    std::lock_guard<std::recursive_mutex> lock(_devices_mutex);

    // Change system id of null device
    if (_devices.find(0) != _devices.end()) {
        auto null_device = _devices.begin()->second[0];
        _devices.erase(0);
        null_device->set_target_system_id(message.sysid);
        _devices.insert(std::pair<uint8_t, std::vector<Device *>>(message.sysid, {null_device}));
    }

    make_new_device(message.sysid, message.compid);

    if (_should_exit) {
        // Don't try to call at() if devices have already been destroyed
        // in descructor.
        return;
    }

    if (message.sysid != 1) {
        LogDebug() << "sysid: " << int(message.sysid);
    }

    // Process the mavlink message of this component
    if (_devices.find(message.sysid) != _devices.end()) {
        for (auto component : _devices[message.sysid])
            if (component->get_target_component_id() == message.compid) {
                component->process_mavlink_message(message);
            }
    }
}

bool DroneCoreImpl::send_message(const mavlink_message_t &message)
{
    std::lock_guard<std::mutex> lock(_connections_mutex);

    for (auto it = _connections.begin(); it != _connections.end(); ++it) {
        if (!(**it).send_message(message)) {
            LogErr() << "send fail";
            return false;
        }
    }

    return true;
}

ConnectionResult DroneCoreImpl::add_any_connection(const std::string &connection_url)
{
    std::string delimiter = "://";
    std::string conn_url(connection_url);
    std::vector<std::string> connection_str;
    size_t pos = 0;
    /*Parse the connection url to get Protocol, IP or Serial dev node
      and port number or baudrate as per the URL definition
    */
    for (int i = 0; i < 2; i++) {
        pos = conn_url.find(delimiter);
        if (pos != std::string::npos) {
            connection_str.push_back(conn_url.substr(0, pos));
            // Erase the string which is parsed already
            conn_url.erase(0, pos + delimiter.length());
            if (conn_url == "") {
                break;
            }
            delimiter = ":";
        }
    }
    connection_str.push_back(conn_url);
    /* check if the protocol is Network protocol or Serial */
    if (connection_str.at(0) != "serial") {
        int port = 0;
        if (connection_str.at(2) != "") {
            port = std::stoi(connection_str.at(2));
        }
        return add_link_connection(connection_str.at(0), connection_str.at(1),
                                   port);
    } else {
        if (connection_str.at(1) == "") {
            return add_serial_connection(DroneCore::DEFAULT_SERIAL_DEV_PATH,
                                         DroneCore::DEFAULT_SERIAL_BAUDRATE);
        } else {
            return add_serial_connection(connection_str.at(1), std::stoi(connection_str.at(2)));
        }
    }
}

ConnectionResult DroneCoreImpl::add_link_connection(const std::string &protocol,
                                                    const std::string &ip, const int port)
{
    int local_port_number = 0;
    std::string local_ip = ip;
    if (port == 0) {
        if (ip != "") {
            local_port_number = std::stoi(ip);
            /* default ip for tcp if only port number is specified */
            local_ip = "127.0.0.1";
        }
    } else {
        local_port_number = port;
    }

    if (protocol == "udp") {
        return add_udp_connection(local_port_number);
    } else { //TCP connection
        return add_tcp_connection(local_ip, local_port_number);
    }
}

ConnectionResult DroneCoreImpl::add_udp_connection(const int local_port_number)
{
    Connection *new_connection = new UdpConnection(this, local_port_number);
    ConnectionResult ret = new_connection->start();

    if (ret != ConnectionResult::SUCCESS) {
        delete new_connection;
        return ret;
    }

    add_connection(new_connection);
    return ConnectionResult::SUCCESS;
}

void DroneCoreImpl::add_connection(Connection *new_connection)
{
    std::lock_guard<std::mutex> lock(_connections_mutex);
    _connections.push_back(new_connection);
}

ConnectionResult DroneCoreImpl::add_tcp_connection(const std::string &remote_ip,
                                                   const int remote_port)
{
    Connection *new_connection = new TcpConnection(this, remote_ip, remote_port);
    ConnectionResult ret = new_connection->start();

    if (ret != ConnectionResult::SUCCESS) {
        delete new_connection;
        return ret;
    }

    add_connection(new_connection);
    return ConnectionResult::SUCCESS;
}

ConnectionResult DroneCoreImpl::add_serial_connection(const std::string &dev_path,
                                                      const int baudrate)
{
#if !defined(WINDOWS) && !defined(APPLE)
    Connection *new_connection = new SerialConnection(this, dev_path, baudrate);
    ConnectionResult ret = new_connection->start();

    if (ret != ConnectionResult::SUCCESS) {
        delete new_connection;
        return ret;
    }

    add_connection(new_connection);
    return ConnectionResult::SUCCESS;
#else
    UNUSED(dev_path);
    UNUSED(baudrate);
    return ConnectionResult::NOT_IMPLEMENTED;
#endif
}

const std::vector<uint64_t> &DroneCoreImpl::get_device_uuids() const
{
    // This needs to survive the scope but we need to clean it up.
    static std::vector<uint64_t> uuids = {};
    uuids.clear();

    for (auto system_it = _devices.begin(); system_it != _devices.end(); ++system_it) {
        // All components will have same UUID, so get it from first component.
        auto uuid = system_it->second[0]->get_target_uuid();
        if (uuid != 0) {
            uuids.push_back(uuid);
        }
    }

    return uuids;
}

Device &DroneCoreImpl::get_autopilot()
{
    std::lock_guard<std::recursive_mutex> lock(_devices_mutex);

    // If called without uuid, we expect to have only one autopilot conneted.
    if (_devices.size() == 1) {
        for (auto component : _devices.begin()->second) {
            if (component->get_target_component_id() == MAV_COMP_ID_AUTOPILOT1) {
                return *component;
            }
        }
    }

    if (_devices.size() > 1) {
        LogErr() << "Error: more than one device found:";

        // Just return first autopilot instead of failing.
        for (auto component : _devices.begin()->second)
            if (component->get_target_component_id() == MAV_COMP_ID_AUTOPILOT1) {
                return *component;
            }
    }

    LogErr() << "Error: no device found.";
    uint8_t system_id = 0, component_id = 0;
    make_new_device(system_id, component_id);
    return *_devices[system_id][component_id];
}

Device &DroneCoreImpl::get_autopilot(const uint64_t uuid)
{
    {
        std::lock_guard<std::recursive_mutex> lock(_devices_mutex);
        // TODO: make a cache map for this.
        for (auto system_it = _devices.begin(); system_it != _devices.end(); ++system_it) {
            for (auto component : system_it->second)
                if (component->get_target_uuid() == uuid &&
                    component->get_target_component_id() == MAV_COMP_ID_AUTOPILOT1) {
                    return *component;
                }
        }

        // We have not found a device with this UUID.
        // TODO: this is an error condition that we ought to handle properly.
        LogErr() << "NO autopilot in UUID: " << uuid;

        // Create a dummy
        uint8_t system_id = 0, component_id = 0;
        make_new_device(system_id, component_id);
        return *_devices[system_id][component_id];
    }
}

bool DroneCoreImpl::is_autopilot_connected() const
{
    std::lock_guard<std::recursive_mutex> lock(_devices_mutex);

    if (_devices.size() == 1) {
        for (auto component : _devices.begin()->second)
            if (component->get_target_component_id() == MAV_COMP_ID_AUTOPILOT1) {
                return component->is_connected();
            }
    }
    return false;
}

bool DroneCoreImpl::is_autopilot_connected(uint64_t uuid) const
{
    std::lock_guard<std::recursive_mutex> lock(_devices_mutex);

    if (_devices.size() == 1) {
        for (auto component : _devices.begin()->second)
            if (component->get_target_uuid() == uuid &&
                component->get_target_component_id() == MAV_COMP_ID_AUTOPILOT1) {
                return component->is_connected();
            }
    }
    return false;
}

bool DroneCoreImpl::make_new_device(uint8_t system_id, uint8_t comp_id)
{
    std::lock_guard<std::recursive_mutex> lock(_devices_mutex);

    if (_should_exit) {
        // When the device got destroyed in the destructor, we have to give up.
        LogWarn() << "Device got destroyed";
        return false;
    }

    // If we already have a component from this system,
    // add it to the list (ie, vector).
    if (_devices.find(system_id) != _devices.end()) {
        for (auto component : _devices[system_id]) {
            // Don't add if the component already exists.
            if (component->get_target_component_id() == comp_id) {
                return false;
            }
        }
        // Create a device
        auto new_device = new Device(this, system_id, comp_id);
        _devices[system_id].push_back(new_device);
        return true;
    }

    auto new_device = new Device(this, system_id, comp_id);
    _devices.insert(std::pair<uint8_t, std::vector<Device *>>(system_id, {new_device}));
    return true;
}

void DroneCoreImpl::notify_on_discover(uint64_t uuid, uint8_t component_id)
{
    LogDebug() << "Discovered component " << int(component_id) << " on vehicle " << uuid;
    if (_on_discover_callback != nullptr) {
        _on_discover_callback(uuid, component_id);
    }
}

void DroneCoreImpl::notify_on_timeout(uint64_t uuid, uint8_t component_id)
{
    LogDebug() << "Lost component " << int(component_id) << " on vehicle " << uuid;
    if (_on_timeout_callback != nullptr) {
        _on_timeout_callback(uuid, component_id);
    }
}

void DroneCoreImpl::register_on_discover(const DroneCore::event_callback_t callback)
{
    _on_discover_callback = callback;
}

void DroneCoreImpl::register_on_timeout(const DroneCore::event_callback_t callback)
{
    _on_timeout_callback = callback;
}

} // namespace dronecore
