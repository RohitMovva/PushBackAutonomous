#include "intake.hpp"

Intake::Intake(pros::Motor& lower, pros::Motor& middle, pros::Motor& upper, EnhancedDigitalOut& stopper_ear, EnhancedDigitalOut& ear,
                pros::Optical& optical_sensor)
    : lower_intake_(lower), middle_intake_(middle), upper_intake_(upper), optical_sensor_(optical_sensor), 
      stopper_ear_(stopper_ear), ear_(ear), detection_task_(nullptr), target_color_(nullptr), intake_state_(0),
      color_sorting_active_(false), ejecting_(false) {
    
    // Initialize optical sensor settings
    optical_sensor_.set_integration_time(20.0);
    optical_sensor_.set_led_pwm(100);
}

Intake::~Intake() {
    stop_color_sorting();
}

void Intake::set_state(int state) {
    intake_state_ = state;
    
    // Only apply motor changes if not currently ejecting
    if (!ejecting_.load()) {
        apply_state_motors();
    }
}

int Intake::get_state() const {
    return intake_state_;
}

void Intake::apply_state_motors() {
    // =============================================================================
    // MOTOR STATE CONFIGURATION AREA - DEFINE YOUR STATE BEHAVIORS HERE
    // =============================================================================
    
    switch (intake_state_) {
        case 0:
            // Stop all motors
            lower_intake_.move_voltage(0);
            middle_intake_.move_voltage(0);
            upper_intake_.move_voltage(0);
            break;
            
        case 1: // Intake
            lower_intake_.move_velocity(12000);
            middle_intake_.move_velocity(12000);
            upper_intake_.move_velocity(12000);
            stopper_ear_.set_value(true);
            break;

        case 2: // Outtake to low goal
            lower_intake_.move_voltage(-12000);
            middle_intake_.move_voltage(-12000);
            upper_intake_.move_voltage(-12000);
            break;
            
        case 3: // Outtake to middle goal
            lower_intake_.move_voltage(12000);
            middle_intake_.move_velocity(12000);
            upper_intake_.move_voltage(-1500);
            break;

        case 4: // Outtake to high goal
            lower_intake_.move_voltage(12000);
            middle_intake_.move_voltage(12000);
            upper_intake_.move_voltage(12000);
            stopper_ear_.set_value(false);
            break;

        default:
            // Default case - stop all motors
            lower_intake_.move_voltage(0);
            upper_intake_.move_voltage(0);
            middle_intake_.move_voltage(0);
            break;
    }
    
    // =============================================================================
    // END MOTOR STATE CONFIGURATION AREA
    // =============================================================================
}

void Intake::start_color_sorting(const char* target_color, const ColorInfo& colors) {
    // Stop any existing color sorting
    stop_color_sorting();
    
    target_color_ = target_color;
    color_params_ = colors;
    color_sorting_active_.store(true);
    
    // Create and start the color detection task
    detection_task_ = new pros::Task(color_detection_task, this, "Color Detection");
}

void Intake::stop_color_sorting() {
    if (detection_task_ != nullptr) {
        color_sorting_active_.store(false);
        ejecting_.store(false);
        
        detection_task_->remove();
        delete detection_task_;
        detection_task_ = nullptr;
        
        // Resume normal state-based motor control
        apply_state_motors();
    }
}

bool Intake::is_color_sorting_active() const {
    return color_sorting_active_.load();
}

bool Intake::is_ejecting() const {
    return ejecting_.load();
}

void Intake::color_detection_task(void* param) {
    Intake* intake_system = static_cast<Intake*>(param);
    intake_system->color_detection_loop();
}

void Intake::color_detection_loop() {
    int detection_counter = 0; // Just for edge cases with weird readings
    while (color_sorting_active_.load()) {
        double hue = optical_sensor_.get_hue();
        double saturation = optical_sensor_.get_saturation();
        pros::lcd::print(0, "Hue: %.2f, Saturation: %.2f", hue, saturation);
        
        bool wrong_color_detected = false;
        
        if (hue >= 999.0) {
            // No valid color detected
            pros::delay(10);
            continue;
        }

        if (saturation > color_params_.saturation_threshold) {
            if (strcmp(target_color_, "red") == 0) {

                if (hue >= color_params_.blue_hue_min && hue <= color_params_.blue_hue_max) {
                    detection_counter++;
                } else {
                    detection_counter = 0;
                }
            } else {
                if (hue >= color_params_.red_hue_min || hue <= color_params_.red_hue_max) {
                    detection_counter++;
                }
                else {
                    detection_counter = 0;
                }
            }
        }
        
        if (detection_counter >= 3) { // Actual object
            // pros::delay();  // Detection delay
            
            ejecting_.store(true);
            
            upper_intake_.move_voltage(-12000);
            pros::delay(150);
            
            upper_intake_.move_voltage(12000);
            
            ejecting_.store(false);
            
            apply_state_motors();
        }
        
        pros::delay(10);
    }
}