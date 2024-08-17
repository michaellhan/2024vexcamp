#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/motors.h"


using namespace pros;

MotorGroup left_dt({-14,-18,15}, v5::MotorGears::blue, v5::MotorUnits::rotations);
MotorGroup right_dt({13,11,-20}, v5::MotorGears::blue, v5::MotorUnits::rotations);
Motor intake(16, v5::MotorGears::blue, v5::MotorUnits::rotations);
Motor conveyorbelt(17, v5::MotorGears::blue, v5::MotorUnits::rotations);
adi::DigitalOut mogo ('C');
Controller controller(E_CONTROLLER_MASTER);

void stop_motors() {
    left_dt.move_voltage(0);
    right_dt.move_voltage(0);
}

//this means that it will go straight
void drivetime(int time, int power) {
    left_dt.move_voltage(power);
    right_dt.move_voltage(power);
    pros::delay(time);
    stop_motors();

}

void turntime (int time, int right, int power) {
    left_dt.move_voltage(power * right);
    right_dt.move_voltage(-power * right);
    pros::delay(time);
    stop_motors();
}


void on_center_button() {


}


void initialize() {
    pros::lcd::initialize();
    controller.clear();

	left_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
	right_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

	mogo.set_value(true);
}


void disabled() {


}


void competition_initialize() {


}



void autonomous() {
    left_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    right_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

   drivetime (539,-6650);
    pros::delay(300);
    mogo.set_value(0);
    pros::delay(300);

    conveyorbelt.move_voltage(6000);
    pros::delay(900);
    conveyorbelt.move_voltage(0);

    turntime(369, 1, 9000);
    drivetime (345, 8955);
    
    conveyorbelt.move_voltage(5700);
    intake.move_voltage(12000);
    drivetime(100, 8955);
    pros::delay(5000);
  
    turntime(260, 1, 7600);
    drivetime(400, 10500);
    pros::delay(5600);

    turntime(100, 1, 5000);
    turntime(100, -1, 5000);
    turntime(100, 1, 5000);
    turntime(100, -1, 5000);
    
    
    

    


}


void opcontrol() {
    left_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    right_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);  

    bool mogo_clamp = true;

    while (true) {
        
        // left_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        // right_dt.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);        

        
        //if (controller.get_digital(E_CONTROLLER_DIGITAL_A)) {
            //drivetime(1000, 6000);
            //1000ms and 12000 power the bot moves 56-57inches
            //1000ms and 6550 power the bot moves abt 42inches
            //1000ms and 6000 power the bot moves abt 30inches


        double left = ((controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) / 127.0);
        

        double controller_const = 0.984252; // percent power when left joystick is at "resting"
        double sigma = 0.858268; // percent power when left joystick is pulled all the way back
        if (left > controller_const) {
            left = (left - controller_const) / (1 - controller_const);
        } else {
            left = -(controller_const - left) / (controller_const - sigma);
        }

        if (fabs(left) < 0.1) {
            left = 0; // deadzone
        }

        double right = ((controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)) / 127.0);

        double mag = fmax (1.0, fmax(fabs(left+right), fabs(left-right)));

        //  controller.print(0, 0, "%f", mag);

        double left_power = ((left + right) / mag) * 12000 * 0.77;
        double right_power = ((left - right) / mag) * 12000;

        controller.print(0, 0, "%f, %f", left, right);

        
        left_dt.move_voltage(left_power);
        right_dt.move_voltage(right_power);


        if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
            intake.move_voltage(12000);
            conveyorbelt.move_voltage(6000);
        } else if (controller.get_digital(E_CONTROLLER_DIGITAL_R2)) {
            intake.move_voltage(-12000);
            conveyorbelt.move_voltage(-6000);
        } else {
            intake.move_voltage(0);
            conveyorbelt.move_voltage(0);
        }   
        

        if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
            if (mogo_clamp) {
                mogo.set_value(0);
                mogo_clamp = false;
            } else {
                mogo.set_value(1);
                mogo_clamp = true;
            }
        }
        pros::delay(10);
    }


}

