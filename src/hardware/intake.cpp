#include "intake.hpp"

Intake::Intake(pros::Motor& lower, pros::Motor& middle, pros::Motor& upper, EnhancedDigitalOut& stopper_ear, EnhancedDigitalOut& ear, EnhancedDigitalOut& park,
                pros::Optical& optical_sensor, pros::Distance& park_sensor)
    : lower_intake_(lower), middle_intake_(middle), upper_intake_(upper), optical_sensor_(optical_sensor), park_sensor_(park_sensor),
      stopper_ear_(stopper_ear), ear_(ear), park_(park), target_color_(nullptr), intake_state_(0),
      color_sorting_active_(false) {
    
    // park_sensor_.set_led_pwm(100);
    // Initialize optical sensor settings
    optical_sensor_.set_integration_time(20.0);
    optical_sensor_.set_led_pwm(100);
    target_color_ = "none";

    // park_sensor_.set_integration_time(20.0);
}

Intake::~Intake() {
    stop_color_sorting();
}

void Intake::set_state(int state) {
    if (intake_state_ == 5 && state == 0) { // don't stop intaking when parking
        return;
    }
    pros::lcd::print(3, "Intake delay: %d", intake_decay_ms_);
    if (intake_decay_ms_ > 0) {
        return; // Ignore state changes during decay
    }
    if (intake_state_ != state) {
        time_since_state_set_ = 0;
    }
    intake_state_ = state;
}

void Intake::state_decay(int start_state, int end_state, int delay_ms) {
    intake_decay_state_ = end_state;
    intake_decay_ms_ = delay_ms;
    set_state(start_state);
}

int Intake::get_state() const {
    return intake_state_;
}

void Intake::apply_state_motors() {   
    switch (intake_state_) {
        case 0:
            // Stop all motors
            lower_intake_.move_voltage(0);
            middle_intake_.move_voltage(0);
            upper_intake_.move_voltage(0);
            break;
            
        case 1: // Intake
            lower_intake_.move_voltage(intake_max_speed_);
            middle_intake_.move_voltage(intake_max_speed_);
            upper_intake_.move_voltage(intake_max_speed_);
            stopper_ear_.set_value(true);
            break;

        case 2: // Outtake to low goal
            lower_intake_.move_voltage(-intake_max_speed_);
            middle_intake_.move_voltage(-intake_max_speed_);
            upper_intake_.move_voltage(-intake_max_speed_);
            break;
            
        case 3: // Outtake to middle goal
            lower_intake_.move_voltage(intake_max_speed_);
            middle_intake_.move_voltage(intake_max_speed_);
            upper_intake_.move_velocity(-50);
            stopper_ear_.set_value(false);
            break;

        case 4: // Outtake to high goal
            lower_intake_.move_voltage(intake_max_speed_);
            middle_intake_.move_voltage(intake_max_speed_);
            upper_intake_.move_voltage(intake_max_speed_);
            stopper_ear_.set_value(false);
            break;

        case 5: // Slow outake to park
            lower_intake_.move_voltage(-6500);
            middle_intake_.move_voltage(-6500);
            upper_intake_.move_voltage(-6500);
            break;

        case 6: // Eject blocks
            lower_intake_.move_voltage(12000);
            middle_intake_.move_voltage(12000);
            upper_intake_.move_voltage(-12000);
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
    target_color_ = target_color;
    color_params_ = colors;
    color_sorting_active_ = true;
}

void Intake::stop_color_sorting() {
    color_sorting_active_ = false;
}

bool Intake::is_color_sorting_active() const {
    return color_sorting_active_;
}

void Intake::update(){
    // Antijam
    if (intake_state_ > 2 && middle_intake_.get_actual_velocity() == 0 && time_since_state_set_ > 250){
        intake_decay_state_ = intake_state_;
        set_state(2);
        intake_decay_ms_ = 250;
    }

    // Color Sort
    double hue = optical_sensor_.get_hue();
    double saturation = optical_sensor_.get_saturation();
    // pros::lcd::print(0, "Hue: %.2f, Saturation: %.2f", hue, saturation);
    
    bool detected = false;
    if (saturation > 0 && color_sorting_active_) {
        if (strcmp(target_color_, "red") == 0) {

            if (hue >= 200 && hue <= 240) {
                detected = true;
            } else {
                detected = false;
            }
        } else if (strcmp(target_color_, "blue") == 0) {
            if (hue >= 310 || hue <= 10) {
                detected = true;
            } else {
                detected = false;
            }
        }
    }
    
    if (detected && color_sorting_active_) { // Actual block
        if (intake_state_ == 3){
            intake_decay_state_ = intake_state_;
            // set_state(2);
            intake_state_ = 2;
            intake_decay_ms_ = 300;
        }
        else {
            intake_decay_state_ = intake_state_;
            intake_decay_ms_ = 20;
            // set_state(6);
            intake_state_ = 6;
        }
                    
        apply_state_motors();
    }

    if (intake_state_ == 5) // parking
    {
        lower_intake_.move_velocity(-100);
        middle_intake_.move_velocity(-100);
        upper_intake_.move_velocity(-100);
        double dist = park_sensor_.get_distance();
        pros::lcd::print(1, "Park Dist: %.2f", dist);
        if (dist < 60) {
            pros::delay(65);
            intake_state_ = 0;
            pros::lcd::print(3, "Parked!");

            park_.set_value(true);
            
            return;
        }
    }

    time_since_state_set_ += 10;

    if (intake_decay_ms_ > 0) {
        intake_decay_ms_ -= 10;
        if (intake_decay_ms_ <= 0) {
            intake_decay_ms_ = 0;
            intake_state_ = intake_decay_state_;
            time_since_state_set_ = 0;
        }
    }



    apply_state_motors();
}

void Intake::park() {
    set_state(5);
    // lower_intake_.move_velocity(-100);
    // middle_intake_.move_velocity(-100);
    // upper_intake_.move_velocity(-100);
    // while (true){
    //     double hue = park_sensor_.get_hue();
    //     pros::lcd::print(1, "Park Hue: %.2f", hue);
    //     pros::lcd::print(2, "%d %d", 
    //     (hue >= color_params_.blue_hue_min && hue <= color_params_.blue_hue_max), 
    //     (hue >= color_params_.red_hue_min || hue <= color_params_.red_hue_max));

    //     pros::lcd::print(5, "%d", hue >= color_params_.red_hue_min);
    //     pros::lcd::print(6, "%d", hue <= color_params_.red_hue_max);

    //     pros::lcd::print(7, "red min: %d", color_params_.red_hue_min);

    //     if ((hue >= 200 && hue <= 260) || 
    //         (hue >= 310|| hue <= 10)) {
    //         pros::delay(150);
    //         intake_state_ = 0;
    //         pros::lcd::print(3, "Parked!");

    //         park_.set_value(true);
            
    //         return;
    //     }
    //     pros::delay(10);
    // }
}