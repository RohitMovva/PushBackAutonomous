#pragma once

#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "hardware/enhanced_digital_out.hpp"
#include <atomic>
#include <cstring>

/**
 * @brief Structure containing color detection parameters
 */
struct ColorInfo {
    int32_t red_hue_min;              ///< Minimum hue value for red detection
    int32_t red_hue_max;              ///< Maximum hue value for red detection
    int32_t blue_hue_min;             ///< Minimum hue value for blue detection
    int32_t blue_hue_max;             ///< Maximum hue value for blue detection
    double_t saturation_threshold;    ///< Minimum saturation for valid color detection
};

/**
 * @brief Integrated intake system with color sorting functionality
 * 
 * This class manages three intake motors (indexer, intake, top_intake) and provides
 * color-based sorting that can temporarily override the current motor state to eject
 * unwanted game pieces.
 */
class Intake {
private:
    // Motors
    pros::Motor& lower_intake_;           ///< Lower intake motor reference
    pros::Motor& middle_intake_;        ///< Middle intake motor reference
    pros::Motor& upper_intake_;        ///< Upper intake motor reference
    EnhancedDigitalOut& stopper_ear_; 
    EnhancedDigitalOut& ear_;
    EnhancedDigitalOut& park_;

    // Color detection
    pros::Optical& optical_sensor_;  ///< Optical sensor reference
    pros::Optical& park_sensor_;     ///< Park sensor reference
    const char* target_color_;       ///< Target color for sorting ("red" or "blue")
    ColorInfo color_params_;         ///< Color detection parameters
    
    // State management
    int intake_state_;               ///< Current intake state (0-10 range)
    int intake_decay_ms_;            ///< Duration to hold temporary states
    int intake_decay_state_;         ///< State to revert to after decay
    int time_since_state_set_;      ///< Time since last state change
    bool color_sorting_active_; ///< Flag indicating if color sorting is running

    int intake_max_speed_ = 12000;
    
    /**
     * @brief Apply motor speeds based on current intake state
     */
    void apply_state_motors();

public:
    /**
     * @brief Constructor for Intake
     * @param indexer Reference to indexer motor
     * @param intake Reference to main intake motor
     * @param top_intake Reference to top intake motor
     * @param stopper_ear Reference to stopper ear digital output
     * @param optical_sensor Reference to optical sensor
     */
    Intake(pros::Motor& indexer, pros::Motor& intake, pros::Motor& top_intake, 
                EnhancedDigitalOut& stopper_ear, EnhancedDigitalOut& ear, EnhancedDigitalOut& park, pros::Optical& optical_sensor, pros::Optical& park_sensor);
    
    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~Intake();
    
    /**
     * @brief Set the intake state
     * @param state Integer state value (0-10 range)
     */
    void set_state(int state);

    /**
     * @brief Temporarily set a state, wait, then revert to another state
     * @param start_state Initial state to set
     * @param end_state State to revert to after delay
     * @param delay_ms Duration to wait in milliseconds
     */
    void state_decay(int start_state, int end_state, int delay_ms);

    /**
     * 
     */
    void set_intake_speed(int speed) {
        intake_max_speed_ = speed;
        apply_state_motors();
    }

    /**
     * 
     */
    void update();
    
    /**
     * @brief Get the current intake state
     * @return Current state value
     */
    int get_state() const;
    
    /**
     * @brief Start color sorting functionality
     * @param target_color Target color to keep ("red" or "blue")
     * @param colors Color detection parameters (optional, uses defaults if not provided)
     */
    void start_color_sorting(const char* target_color, const ColorInfo& colors = {
        310, 10,   // Red hue range
        200, 270,  // Blue hue range  
        0.0        // Minimum saturation threshold
    });
    
    /**
     * @brief Stop color sorting functionality
     */
    void stop_color_sorting();

    void set_shift(bool state) {
        if (state){
            intake_max_speed_ = 6000;
        } else {
            intake_max_speed_ = 12000;
        }
    }
    
    /**
     * @brief Check if color sorting is currently active
     * @return true if color sorting is running, false otherwise
     */
    bool is_color_sorting_active() const;

    void park();
};