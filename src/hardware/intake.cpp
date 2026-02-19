#include "intake.hpp"

Intake::Intake(pros::Motor& lower, pros::Motor& middle, pros::Motor& upper, EnhancedDigitalOut& stopper_ear, EnhancedDigitalOut& ear,
                pros::Optical& optical_sensor, bool skills)
    : lower_intake_(lower), middle_intake_(middle), upper_intake_(upper), optical_sensor_(optical_sensor),
      stopper_ear_(stopper_ear), ear_(ear), target_color_(nullptr), intake_state_(0), skills_(skills),
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
    if (state != 3 && (intake_state_ == 3 || intake_state_ == 13)){
        intake_state_ = state;
        intake_decay_ms_ = 0;
        time_since_state_set_ = 0;
    }
    // if ((intake_state_ == 15 || intake_state_ == 16 || intake_state_ == 17)){
    //     return;
    // }

    if (intake_decay_ms_ > 0 && state != 8) {
        return; // Ignore state changes during decay
    }
    
    if (intake_state_ != state) {
        time_since_state_set_ = 0;
    }
    intake_state_ = state;
}

void Intake::state_decay(int start_state, int end_state, int delay_ms) {
    if (start_state == 3 && (intake_state_ == 3 || intake_state_ == 13) && intake_decay_ms_ > 0){
        return;
    }
    pros::lcd::print(1, "dlayy");
    intake_decay_state_ = end_state;
    intake_decay_ms_ = delay_ms;
    set_state(start_state);
}

int Intake::get_state() const {
    return intake_state_;
}

void Intake::apply_state_motors() {   
    pros::lcd::print(0, "intake state: %d", intake_state_);
    if (intake_state_ != 3 && skills_){
        ear_.set_value(false);
    }
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
            // SLOW SKILLS CONTROLS
            if (skills_) {
                lower_intake_.move_velocity(-30);
                upper_intake_.move_velocity(5);

                middle_intake_.move_voltage(-intake_max_speed_);
                stopper_ear_.set_value(true);
                ear_.set_value(true);
                
            } else {
                lower_intake_.move_voltage(-intake_max_speed_);
                upper_intake_.move_velocity(5);

                middle_intake_.move_voltage(-intake_max_speed_);
            }

            break;
            
        case 3: // Outtake to middle goal
            // SLOW SKILLS CONTROLS
            if (skills_){
                lower_intake_.move_voltage(intake_max_speed_);
                middle_intake_.move_velocity(75);
                upper_intake_.move_velocity(-30);
                // upper_intake_.move_voltage(-3000);

                stopper_ear_.set_value(false);
            } else {
                lower_intake_.move_voltage(intake_max_speed_);
                middle_intake_.move_voltage(intake_max_speed_);
                upper_intake_.move_voltage(-intake_max_speed_);
                stopper_ear_.set_value(false);
            }

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

        case 7:
            lower_intake_.move_voltage(intake_max_speed_);
            middle_intake_.move_voltage(intake_max_speed_);
            upper_intake_.move_voltage(intake_max_speed_);
            stopper_ear_.set_value(false);
            break;

        case 8:
            // lower_intake_.move_voltage(-intake_max_speed_);
            upper_intake_.move_velocity(-50);

            middle_intake_.move_voltage(-intake_max_speed_);
            stopper_ear_.set_value(true);
            break;

        case 9:
            lower_intake_.move_voltage(0);
            // middle_intake_.move_velocity(100);
            middle_intake_.move_velocity(-25);
            upper_intake_.move_velocity(-40);
            stopper_ear_.set_value(false);
            break;

        case 10:
            lower_intake_.move_velocity(-75);
            upper_intake_.move_velocity(5);

            middle_intake_.move_voltage(-intake_max_speed_);
            stopper_ear_.set_value(true);

            // ear_.set_value(true);
            break;

        case 11:
            lower_intake_.move_voltage(intake_max_speed_);
            middle_intake_.move_velocity(100);
            upper_intake_.move_velocity(-70);
            stopper_ear_.set_value(false);
            break;

        case 12:
            lower_intake_.move_voltage(0);
            middle_intake_.move_velocity(-75);
            upper_intake_.move_velocity(-40);
            stopper_ear_.set_value(false);
            break;

        case 13:
            lower_intake_.move_voltage(intake_max_speed_);
            middle_intake_.move_velocity(-75);
            upper_intake_.move_velocity(-40);
            stopper_ear_.set_value(false);

            break;

        case 14:
            lower_intake_.move_voltage(intake_max_speed_);
            // middle_intake_.move_velocity(-75);
            // upper_intake_.move_velocity(-40);
            stopper_ear_.set_value(true);

            break;

        case 15:
            lower_intake_.move_voltage(intake_max_speed_);
            middle_intake_.move_velocity(110);
            upper_intake_.move_velocity(-63);
            if (intake_decay_ms_ > 1150){
                middle_intake_.move_voltage(0);
            }
            // upper_intake_.move_voltage(-3000);

            stopper_ear_.set_value(false);
            break;
        
        case 16:
            lower_intake_.move_voltage(intake_max_speed_);
            middle_intake_.move_velocity(100);
            upper_intake_.move_velocity(-50);
            if (intake_decay_ms_ > 400 && intake_decay_ms_ < 650){
                middle_intake_.move_velocity(-60);
            }
            // upper_intake_.move_voltage(-3000);

            stopper_ear_.set_value(false);
            break;
        
        case 17:
            lower_intake_.move_voltage(intake_max_speed_);
            middle_intake_.move_velocity(100);
            // if (intake_decay_ms_ > 600 && intake_decay_ms_ < 850){
            //     middle_intake_.move_velocity(-60);
            // }
            upper_intake_.move_velocity(-18);
            // upper_intake_.move_voltage(-3000);

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
    // if (intake_state_ > 0 && middle_intake_.get_actual_velocity() == 0 && time_since_state_set_ > 250 && intake_state_ != 13 && intake_state_ != 3){
    //     intake_decay_state_ = intake_state_;
    //     if (intake_state_ == 2 || intake_state_ == 10) {
    //         set_state(1);
    //     } else {
    //         set_state(8);
    //     }
    //     intake_decay_ms_ = 170;
    // }

    // if (intake_state_ == 3 && fabs(upper_intake_.get_actual_velocity()) <= 10 && time_since_state_set_ > 250){
    //     intake_decay_state_ = intake_state_;
    //     set_state(12);
    //     intake_decay_ms_ = 200;
    // }


    // if (intake_state_ > 0 && lower_intake_.get_actual_velocity() == 0 && time_since_state_set_ > 250){
    //     intake_decay_state_ = intake_state_;
    //     if (intake_state_ == 2) {
    //         set_state(1);
    //     } else {
    //         set_state(2);
    //     }
    //     intake_decay_ms_ = 250;
    // }

    // Color Sort
    double hue = optical_sensor_.get_hue();
    double saturation = optical_sensor_.get_saturation();
    // pros::lcd::print(0, "Hue: %.2f, Saturation: %.2f", hue, saturation);
    // pros::lcd::print(1, "State: %d", intake_state_);
    pros::lcd::print(1, "dist: %d", optical_sensor_.get_proximity());
    
    bool detected = false;
    if (saturation > 0 && color_sorting_active_ && optical_sensor_.get_proximity() > 100) {
        if (strcmp(target_color_, "red") == 0) {

            if (hue >= 200 && hue <= 240) {
                detected = true;
            } else {
                detected = false;
            }
        } else if (strcmp(target_color_, "blue") == 0) {
            if (hue >= 359 || hue <= 20) {
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

    time_since_state_set_ += 10;

    // intake_decay_ms_ -= 10;
    if (intake_decay_ms_ > 0) {
        intake_decay_ms_ -= 10;
        if (intake_decay_ms_ <= 0) {
            intake_decay_ms_ = 0;
            intake_state_ = intake_decay_state_;
            time_since_state_set_ = 0;
            pros::lcd::print(6, "set state to %d", intake_decay_state_);
            if (skills_ && intake_state_ == 3){
                intake_decay_state_ = 13;
                intake_decay_ms_ = 600;
            }
            else if (skills_ && (intake_state_ == 13 || intake_state_ == 12)){
                intake_decay_state_ = 3;
                intake_decay_ms_ = 100;
            }

            if (skills_ && intake_state_ == 16){
                intake_decay_state_ = 17;
                intake_decay_ms_ = 1200;
            }

            if (skills_ && intake_state_ == 17){
                intake_decay_state_ = 0;
                intake_decay_ms_ = 1500;
            }

            // if (skills_ && intake_state_ == 16){
            //     intake_decay_state_ = 16;
            //     intake_decay_ms_ = 400;
            // }
        }
    }

    if (intake_state_ == 7 && (hue >= 200 && hue <= 250)){
        set_state(8);
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