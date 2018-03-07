#include <iostream>
#include "integration_test_helper.h"
#include "global_include.h"
#include "dronecore.h"

using namespace dronecore;
using namespace std::chrono;
using namespace std::this_thread;
using namespace std::placeholders;
static size_t _component_count = 0;

static void on_component_discover(uint64_t uuid, uint8_t component_id)
{
    if (uuid && int(component_id)) {
        _component_count++;
    }
    std::cout << "Discovered component " << int(component_id)
              << " on vehicle " << uuid << std::endl;
}

TEST_F(SitlTest, DiscoverOneAutopilotAndCamera)
{
    DroneCore connection;
    std::cout << "started" << std::endl;

    connection.add_udp_connection();
    connection.register_on_discover(std::bind(on_component_discover, _1, _2));

    sleep_for(seconds(5));
    EXPECT_EQ(_component_count, 2);
    std::cout << "exiting" << std::endl;
}
