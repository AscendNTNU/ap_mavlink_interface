#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>

#define AP_MAV_INFO(msg) std::cerr << "Info: " << msg << std::endl
#define AP_MAV_WARN(msg) std::cerr << "Warning: " << msg << std::endl
#define AP_MAV_ERR(msg) std::cerr << "Error: " << msg << std::endl

struct State {
    enum Value {
        Idle,
        Connected,
        Armed,
        OffboardUnarmed,
        Ready,
        PositionFollow,
        AttitudeRateFollow,
    };

    State() = delete;
    constexpr State(Value state) : value(state) {}

    operator Value() const { return value; }
    explicit operator bool() = delete;

    bool offboard() {
        return value == Ready || value == OffboardUnarmed || value == PositionFollow || value == AttitudeRateFollow;
    }

    bool armed() {
        return value == Ready || value == Armed || value == PositionFollow || value == AttitudeRateFollow;
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
    std::function<void()> ready_cb;

private:
    int stop_offboard() {
        //if (offboard->is_active()) {
        if (state.offboard()) {
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
        if (state != State::Idle) {
            AP_MAV_ERR("Not in idle state");
            return 1;
        }

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
        offboard = std::make_shared<mavsdk::Offboard>(system);

        // We want to listen to the altitude of the drone at 1 Hz.
        const mavsdk::Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
        if (set_rate_result != mavsdk::Telemetry::Result::SUCCESS) {
            AP_MAV_ERR("Could not set telemetry rate");
            return 1;
        }

        telemetry->armed_async([this](bool armed) {
            if (armed && !this->state.armed()) {
                AP_MAV_INFO("Drone was armed");
                this->offboard->set_attitude({0, 0, 0, 0});
                if (this->state.offboard()) {
                    this->state = State::Ready;
                    this->ready_cb();
                }
                else {
                    this->state = State::Armed;
                }
            }
            else if (!armed && this->state.armed()) {
                AP_MAV_INFO("Drone was unarmed");
                if (this->state.offboard()) {
                    this->state = State::OffboardUnarmed;
                }
                else {
                    this->state = State::Connected;
                }
            }
        });

        telemetry->flight_mode_async([this](mavsdk::Telemetry::FlightMode flight_mode) {
            if (flight_mode == mavsdk::Telemetry::FlightMode::OFFBOARD && !this->state.offboard()) {
                AP_MAV_INFO("Drone entered offboard mode");
                // FIXME: Should we reset attitude here or not?
                // this->offboard->set_attitude({0, 0, 0, 0});
                if (this->state.armed()) {
                    this->state = State::Ready;
                    this->ready_cb();
                }
                else {
                    this->state = State::OffboardUnarmed;
                }
            }
            else if (flight_mode != mavsdk::Telemetry::FlightMode::OFFBOARD && this->state.offboard()) {
                AP_MAV_INFO("Drone exited offboard mode");
                if (this->state.armed()) {
                    this->state = State::Armed;
                }
                else {
                    this->state = State::Connected;
                }
            }
        });

        // We want to set a offboard pos, so that we can enter offboard mode later
        offboard->set_attitude({0, 0, 0, 0});

        state = State::Connected;

        return 0;
    }

    int arm() {
        if (state != State::Connected) {
            AP_MAV_ERR("Not in connected state");
            return 1;
        }

        // Check if vehicle is ready to arm
        while (telemetry->health_all_ok() != true) {
            AP_MAV_INFO("Getting ready to arm");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Arm vehicle
        AP_MAV_INFO("Arming...");
        const mavsdk::Action::Result arm_result = action->arm();

        return 0;
    }

    int position_follow(float x0, float y0, float z0, float yaw0) {
        offboard->set_position_ned({x0, y0, -z0, yaw0});

        if (!state.offboard()) {
            AP_MAV_WARN("Not in offboard mode");
            return 1;
        }

        state = State::PositionFollow;

        return 0;
    }

    int attitude_rate_follow(float roll_r, float pitch_r, float yaw_r, float thrust) {
        offboard->set_attitude_rate({roll_r, pitch_r, yaw_r, thrust});

        if (!state.offboard()) {
            AP_MAV_WARN("Not in offboard mode");
            return 1;
        }

        state = State::AttitudeRateFollow;

        return 0;
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

    int start_offboard() {
        //offboard->set_attitude({0, 0, 0, 0});
        mavsdk::Offboard::Result offboard_result = offboard->start();

        if (offboard_result != mavsdk::Offboard::Result::SUCCESS) {
            AP_MAV_ERR("Could not enter offboard state");
            return 1;
        }

        AP_MAV_INFO("Entered offboard mode");
        return 0;
    }

    void on_ready(std::function<void()> fn) {
        ready_cb = fn;
    }
};
