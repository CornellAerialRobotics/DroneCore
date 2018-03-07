#include <iostream>
#include "integration_test_helper.h"
#include "dronecore.h"
#include "plugins/info/info.h"

using namespace dronecore;
using namespace std::placeholders;

static void on_discover(uint64_t uuid, uint8_t component_id);
static bool _discovered_device = false;

TEST_F(SitlTest, Info)
{
    DroneCore dc;

    ConnectionResult ret = dc.add_udp_connection();
    ASSERT_EQ(ret, ConnectionResult::SUCCESS);

    dc.register_on_discover(std::bind(&on_discover, _1, _2));

    while (!_discovered_device) {
        std::cout << "waiting for device to appear..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto info = std::make_shared<Info>(&dc.autopilot());

    for (unsigned i = 0; i < 3; ++i) {
        Info::Version version = info->get_version();

        std::cout << "Flight version: "
                  << version.flight_sw_major << "."
                  << version.flight_sw_minor << "."
                  << version.flight_sw_patch << " ("
                  << std::string(version.flight_sw_git_hash) << ")" << std::endl;
        std::cout << "Flight vendor version: "
                  << version.flight_sw_vendor_major << "."
                  << version.flight_sw_vendor_minor << "."
                  << version.flight_sw_vendor_patch << std::endl;
        std::cout << "OS version: "
                  << version.os_sw_major << "."
                  << version.os_sw_minor << "."
                  << version.os_sw_patch << " ("
                  << std::string(version.os_sw_git_hash) << ")" << std::endl;

        EXPECT_NE(version.flight_sw_major, 0);

        // FIXME: This is currently 0.
        //EXPECT_NE(version.os_sw_major, 0);


        Info::Product product = info->get_product();

        std::cout << "Vendor: " << product.vendor_name << std::endl;
        std::cout << "Product: " << product.product_name << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void on_discover(uint64_t uuid, uint8_t component_id)
{
    std::cout << "Found device with component ID " << component_id
              << " on vehicle whose UUID: " << uuid << std::endl;
    _discovered_device = true;
}
