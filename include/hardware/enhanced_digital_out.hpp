#ifndef ENHANCED_DIGITAL_OUT_HPP
#define ENHANCED_DIGITAL_OUT_HPP

#include "api.h"

class EnhancedDigitalOut : public pros::adi::DigitalOut {
public:
    /**
     * Creates a new EnhancedDigitalOut object on the specified port
     *
     * @param port The ADI port number (1-8)
     * @param initial_state The initial state to set the pin to (default: false)
     */
    explicit EnhancedDigitalOut(std::uint8_t port, bool initial_state = false);

    /**
     * Toggles the digital output state
     *
     * @return true if the operation was successful
     */
    bool toggle();

    /**
     * Gets the current state of the digital output
     *
     * @return The current state (true = HIGH, false = LOW)
     */
    bool get_state() const;

    /**
     * Pulses the output for a specified duration
     *
     * @param pulse_duration_ms The duration of the pulse in milliseconds
     * @param new_state The state to pulse to (default: opposite of current state)
     * @return true if the operation was successful
     */
    bool pulse(std::uint32_t pulse_duration_ms, bool new_state = true);

    /**
     * Toggles the output based on a button input, only toggling on the rising edge
     * (when button changes from not pressed to pressed)
     *
     * @param button_pressed The current state of the button
     * @return true if a toggle occurred
     */
    bool input_toggle(bool button_pressed);

private:
    bool current_state;      // Tracks the current state of the digital output
    bool last_button_state;  // Tracks the previous state of the button input
};

#endif // ENHANCED_DIGITAL_OUT_HPP