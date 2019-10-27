#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <iostream>
#include <thread>

//#include <fsm.hpp>

#define AP_MAV_INFO(msg) std::cerr << "Info: " << msg << std::endl
#define AP_MAV_ERR(msg) std::cerr << "Error: " << msg << std::endl

struct State {
    enum Value {
        Idle,
        Connected,
        Armed,
        PositionFollow,
        AttitudeFollow,
        Halted
    };

    State() = delete;
    constexpr State(Value state) : value(state) {}

    operator Value() const { return value; }
    explicit operator bool() = delete;

    bool ready_to_fly() {
        return value == Armed || value == PositionFollow || value == AttitudeFollow || value == Halted;
    }

    bool is_offboard() {
        return value == PositionFollow || value == AttitudeFollow;
    }
private:
    Value value;
};

class ApMavController {
    State state = State::Idle;
    mavsdk::Mavsdk dc;
    std::shared_ptr<mavsdk::Action> action;
    std::shared_ptr<mavsdk::Telemetry> telemetry;
    std::shared_ptr<mavsdk::Offboard> offboard;

private:
    int start_offboard() {
        if (state == State::Armed) {
            mavsdk::Offboard::Result offboard_result = offboard->start();

            if (offboard_result != mavsdk::Offboard::Result::SUCCESS) {
                AP_MAV_ERR("Could not enter offboard state");
                return 1;
            }
        }
        return 0;
    }

    int stop_offboard() {
        //if (offboard->is_active()) {
        if (state.is_offboard()) {
            mavsdk::Offboard::Result offboard_result = offboard->stop();

            if (offboard_result != mavsdk::Offboard::Result::SUCCESS) {
                AP_MAV_ERR("Could not enter offboard state");
                return 1;
            }
        }
        return 0;
    }

public:
    int connect(std::string connection_url) {
        mavsdk::ConnectionResult conn_res = dc.add_any_connection(connection_url);
        if (conn_res != mavsdk::ConnectionResult::SUCCESS) {
            AP_MAV_ERR("Connection failed");
        }

        bool discovered_system = false;
        mavsdk::System &system = dc.system();

        dc.register_on_discover([&discovered_system](uint64_t) {
            AP_MAV_INFO("Found system");
            discovered_system = true;
        });

        // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
        // seconds.
        std::this_thread::sleep_for(std::chrono::seconds(2));

        if (!discovered_system) {
            AP_MAV_ERR("No system found");
            return 1;
        }

        telemetry = std::make_shared<mavsdk::Telemetry>(system);
        action = std::make_shared<mavsdk::Action>(system);

        // We want to listen to the altitude of the drone at 1 Hz.
        const mavsdk::Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
        if (set_rate_result != mavsdk::Telemetry::Result::SUCCESS) {
            AP_MAV_ERR("Could not set telemetry rate");
            return 1;
        }

        state = State::Connected;

        return 0;
    }

    int arm() {
        if (state != State::Connected) {
            AP_MAV_ERR("Not in connected state");
            return 1;
        }

        /*
        telemetry->position_async([](Telemetry::Position position) {
            std::cout << TELEMETRY_CONSOLE_TEXT // set to blue
                      << "Altitude: " << position.relative_altitude_m << " m"
                      << NORMAL_CONSOLE_TEXT // set to default color again
                      << std::endl;
        });
        */

        // Check if vehicle is ready to arm
        while (telemetry->health_all_ok() != true) {
            AP_MAV_INFO("Getting ready to arm");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Arm vehicle
        AP_MAV_INFO("Arming...");
        const mavsdk::Action::Result arm_result = action->arm();

        if (arm_result != mavsdk::Action::Result::SUCCESS) {
            AP_MAV_ERR("Arming failed");
            return 1;
        }

        state = State::Armed;
        return 0;
    }

    int position_follow(float x0, float y0, float z0, float yaw0) {
        if (!state.ready_to_fly()) {
            AP_MAV_ERR("Not in correct state");
            return 1;
        }

        offboard->set_position_ned({x0, y0, -z0, yaw0});

        if(start_offboard()) {
            return 1;
        }

        state = State::PositionFollow;
    }

    int attitude_follow(float roll0, float pitch0, float yaw0, float thrust0) {
        if (!state.ready_to_fly()) {
            AP_MAV_ERR("Not in correct state");
            return 1;
        }

        offboard->set_attitude({roll0, pitch0, yaw0, thrust0});

        if(start_offboard()) {
            return 1;
        }

        state = State::AttitudeFollow;
    }

    int set_position(float x, float y, float z, float yaw) {
        if (state != State::PositionFollow) {
            AP_MAV_ERR("Not in position follow mode");
            return 1;
        }

        offboard->set_position_ned({x, y, -z, yaw});

        return 0;
    }

    int set_attitude(float roll, float pitch, float yaw, float thrust) {
        if (state != State::AttitudeFollow) {
            AP_MAV_ERR("Not in attitude follow mode");
            return 1;
        }

        offboard->set_attitude({roll, pitch, yaw, thrust});
    }

    int land() {
        if (stop_offboard()) {
            return 1;
        }

        AP_MAV_INFO("Landing...");
        const mavsdk::Action::Result land_result = action->land();
        if (land_result != mavsdk::Action::Result::SUCCESS) {
            AP_MAV_ERR("Land failed");
            return 1;
        }

        // Check if vehicle is still in air
        while (telemetry->in_air()) {
            AP_MAV_INFO("Landing");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        AP_MAV_INFO("Landed");

        const mavsdk::Action::Result disarm_result = action->disarm();

        if (disarm_result != mavsdk::Action::Result::SUCCESS) {
            AP_MAV_ERR("Disarm failed");
            return 1;
        }

        state = State::Connected;

        return 0;
    }

    int halt() {
        if (!state.ready_to_fly()) {
            AP_MAV_ERR("Not in a flying state");
            return 1;
        }

        mavsdk::Telemetry::PositionNED pos = telemetry->position_velocity_ned().position;

        offboard->set_attitude({pos.north_m, pos.east_m, pos.down_m, 0});

        if(start_offboard()) {
            return 1;
        }

        state = State::Halted;

        return 0;
    }
};

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

int main(int argc, char **argv)
{
    ApMavController controller;
    std::string connection_url;

    if (argc == 2) {
        connection_url = argv[1];
        if (controller.connect(connection_url)) {
            return 1;
        }
    } else {
        usage(argv[0]);
        return 1;
    }

    if (controller.arm()) {
        return 1;
    }

    controller.set_position(0, 0, 1, 0);

    std::this_thread::sleep_for(std::chrono::seconds(3));

    controller.land();

    return 0;
}
