#pragma once
#include <vector>
#include <cstdint>
#include <chrono>

// Mock PROS namespace
namespace pros {
    // Add a static time offset that we can control
    namespace {
        static int64_t time_offset_ms = 0;
    }

namespace pros {
    // Add a static time offset that we can control
    namespace {
        static int64_t time_offset_ms = 0;
    }

    // Add control functions
    inline void set_millis(uint32_t ms) {
        time_offset_ms = ms - std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
    }

    inline void increment_millis(int32_t ms) {
        time_offset_ms += ms;
    }

    // Modified millis() function to use the offset
    inline uint32_t millis() {
        return static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count() + time_offset_ms
        );
    }

    class Motor {
    public:
        Motor(int port) : position_(0) {}
        void set_position(double position) { position_ = position; }
        double get_position() const { return position_; }
        void tare_position() { position_ = 0; }
    private:
        double position_;
    };

    class MotorGroup {
    public:
        MotorGroup(const std::vector<Motor*>& motors) : motors_(motors) {}
        std::vector<double> get_position_all() const {
            std::vector<double> positions;
            for (const auto& motor : motors_) {
                positions.push_back(motor->get_position());
            }
            return positions;
        }
        void tare_position() {
            for (auto& motor : motors_) {
                motor->tare_position();
            }
        }
    private:
        std::vector<Motor*> motors_;
    };

    class Encoder {
    public:
        Encoder(int port_top, int port_bottom) : value_(0) {}
        void reset() { value_ = 0; }
        int32_t get_value() const { return value_; }
        void set_value(int32_t value) { value_ = value; }
    private:
        int32_t value_;
    };

    class Imu {
    public:
        struct gyro_rate_s { double z; };
        
        Imu(int port) : rotation_(0), gyro_rate_{0} {}
        void tare_rotation() { rotation_ = 0; }
        double get_rotation() const { return rotation_; }
        void set_rotation(double rotation) { rotation_ = rotation; }
        gyro_rate_s get_gyro_rate() const { return gyro_rate_; }
        void set_gyro_rate(double rate) { gyro_rate_.z = rate; }
    private:
        double rotation_;
        gyro_rate_s gyro_rate_;
    };
}