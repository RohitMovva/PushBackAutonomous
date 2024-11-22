// enhanced_digital_out.cpp
#include "hardware/enhanced_digital_out.h"

EnhancedDigitalOut::EnhancedDigitalOut(std::uint8_t port, bool initial_state)
    : pros::adi::DigitalOut(port, initial_state), 
      current_state(initial_state),
      last_button_state(false) {
}

bool EnhancedDigitalOut::toggle() {
    current_state = !current_state;
    return set_value(current_state);
}

bool EnhancedDigitalOut::get_state() const {
    return current_state;
}

bool EnhancedDigitalOut::pulse(std::uint32_t pulse_duration_ms, bool new_state) {
    bool original_state = current_state;
    bool pulse_state = new_state;
    
    if (!set_value(pulse_state)) {
        return false;
    }
    current_state = pulse_state;
    
    pros::delay(pulse_duration_ms);
    
    if (!set_value(original_state)) {
        return false;
    }
    current_state = original_state;
    
    return true;
}

bool EnhancedDigitalOut::input_toggle(bool button_pressed) {
    // Check for rising edge (button was just pressed)
    bool rising_edge = button_pressed && !last_button_state;
    
    // Update the stored button state
    last_button_state = button_pressed;
    
    // If we detect a rising edge, toggle the output
    if (rising_edge) {
        return toggle();
    }
    
    return false;
}