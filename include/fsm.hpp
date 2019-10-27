#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#define AP_MAV_ERR(err) std::cerr << err << std::endl

namespace ascend {
    enum State {
        Idle,
        Takeoff,
        PositionFollow,
        RawAttitude,
        Halted,
        Landing
    };

    class ApMavController {
        mavsdk::Mavsdk dc;
        mavsdk::System& system;


    public:
        int connect(std::string connection_url) {
            mavsdk::ConnectionResult conn_res = dc.add_any_connection(connection_url);
            if (conn_res != mavsdk::ConnectionResult::SUCCESS) {
                AP_MAV_ERR("Connection failed");

            }
            
        }
    };
};
