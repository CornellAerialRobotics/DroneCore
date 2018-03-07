#include <iostream>
#include "integration_test_helper.h"
#include "global_include.h"
#include "dronecore.h"

using namespace dronecore;
using namespace std::placeholders; // for _1

static bool _discovered_device = false;
static bool _timeouted_device = false;
static uint64_t _uuid = 0;
static uint8_t _component_id = 0;

void on_discover(uint64_t uuid, uint8_t component_id);
void on_timeout(uint64_t uuid, uint8_t component_id);

TEST_F(SitlTest, AsyncConnect)
{
    DroneCore dc;

    ASSERT_EQ(dc.add_udp_connection(), ConnectionResult::SUCCESS);

    dc.register_on_discover(std::bind(&on_discover, _1, _2));
    dc.register_on_timeout(std::bind(&on_timeout, _1, _2));

    while (!_discovered_device) {
        std::cout << "waiting for device to appear..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Let params stabilize before shutting it down.
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Call gtest to shut down SITL.
    TearDown();

    while (!_timeouted_device) {
        std::cout << "waiting for device to disappear..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void on_discover(uint64_t uuid, uint8_t component_id)
{
    std::cout << "Found component " << int(component_id)
              << " on device with UUID: " << uuid << std::endl;
    _discovered_device = true;
    _uuid = uuid;
    _component_id = component_id;
    // The UUID and Component ID should not be 0.
    EXPECT_NE(_uuid, 0);
    EXPECT_NE(_component_id, 0);
}

void on_timeout(uint64_t uuid, uint8_t component_id)
{
    std::cout << "Lost component " << int(component_id)
              << " on device with UUID: " << uuid << std::endl;
    _timeouted_device = true;

    // The UUID and Component ID should still be the same.
    EXPECT_EQ(_uuid, uuid);
    EXPECT_EQ(_component_id, component_id);
}
