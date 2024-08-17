#include "main.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"


using namespace pros;
MotorGroup left_dt({-9, -10}, v5::MotorGears::blue, v5::MotorUnits::rotations);
MotorGroup right_dt({2, 6}, v5::MotorGears::blue, v5::MotorUnits::rotations);
Motor intake_front(5, v5::MotorGears::blue, v5::MotorUnits::rotations);
Motor intake_back(1, v5::MotorGears::blue, v5::MotorUnits::rotations);
adi::DigitalOut mogo_clamp('C');
Controller controller(E_CONTROLLER_MASTER);

/**/

void stop_motors() {
  left_dt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_dt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  // left_dt.brake();
  // right_dt.brake();
  left_dt.move_voltage(0); 
  right_dt.move_voltage(0);
}
// time in milliseconds

void drive_time(int time, int power) {
  double wheell = 0.82;
  right_dt.move_voltage(power);
  delay(25);
  left_dt.move_voltage(power * wheell);
  pros::delay(time);
  stop_motors();
}
// num = 1 for right, num= -1 for left
void turn_time(int time, int num, int power) {
  left_dt.move_voltage(power * num);
  right_dt.move_voltage(-power * num);
  pros::delay(time);
  stop_motors();

}

void on_center_button() {}

void initialize() { 
  pros::lcd::initialize(); 
  left_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
  right_dt.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
  mogo_clamp.set_value(false);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  //Step One: Getting to the closest stake
  drive_time(900, -8000);
  
}
  

void opcontrol() {
  left_dt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_dt.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  bool mogo_clamp_state = false;

  while (true) {
    double left = ((controller.get_analog(ANALOG_LEFT_Y)) / 127.0);
    double right = ((controller.get_analog(ANALOG_RIGHT_X)) / 127.0);
    double wheell = 0.68;
    double wheelr = 1.2;
    double mag = fmax(1.0, fmax(fabs(left + right), fabs(left - right)));
    double left_power = (((left + right) / mag) * 12000) * wheell;
    double right_power = (((left - right) / mag) * 12000) * wheelr;
    left_dt.move_voltage(left_power);
    right_dt.move_voltage(right_power);

    // Some code (intake and controller_digital_r1) isn't defined so there is
    // edititing that has to be done

    if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
      intake_front.move_voltage(12000);
      intake_back.move_voltage(12000);
    } else if (controller.get_digital(E_CONTROLLER_DIGITAL_R2)) {
      intake_front.move_voltage(-12000);
      intake_back.move_voltage(-12000);
    } else {
      intake_front.move_voltage(0);
      intake_back.move_voltage(0);
    }
    
    if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
      if (mogo_clamp_state) {
        mogo_clamp.set_value(0);
        mogo_clamp_state = false;
      } else {
        mogo_clamp.set_value(1);
        mogo_clamp_state = true;
      }
    }
  
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      //drive_time(1000, 4000);
      //drive_time(2000, 8000); - went 69 inches, 68-69 inches, 69 inches, 70 inches
      //drive_time(1000, 8000); - went 32 inches, 32 inches, 34 inches, 34 inches
      //drive_time(1000, 12000); - went 48 inches, 47 inches, 49 inches, 47 inches
      //drive_time(2000, 12000); - went 100 inches, 100 inches, 100 inches, 100 inches
      //drive_time(1000, 2000); - went 5 inches
      //drive_time(1000, 4000); - went 15 inches, went 15 inches

      //turn_time(820, -1, 4000);

      //turn_time(1000, 1, 4000); - went (little more than) 90 degrees, 90 degrees, 90 degrees, 90 degrees
      //turn_time(820, -1, 4000); - went 90 degrees, 90 degrees, 90 degrees, 90 degrees
      //turn_time(820, 1, 4000); - went 90 degrees, 90 degrees, 90 degrees, 90 degrees
      mogo_clamp.set_value(1);
      drive_time(850, -8000);
      mogo_clamp.set_value(0);
			pros::delay(500);
      intake_back.move_voltage(-12000); //first ring
      pros::delay(1500); //fixed angle, it has to be slightly toward the right if you are looking from the perspective of the bot so that the whole thing is centered 
			drive_time(300, 4000);
			drive_time(300, -4000);
			//while (true) {
				//intake_front.move_voltage(1000);
				//pros::delay(1500);
			//}

		
			turn_time(1000, -1, 2000);
			drive_time(1500, 4000);
			//intake_front.move_voltage(1000); //second ring
			//intake_back.move_voltage(-12000);
			//pros::delay(1500); 

			//turn_time(1500, 1, 8000); 
			//drive_time(1500, 8000);
			//turn_time(820, 1, 4000);
			//drive_time(200, 4000);
			//intake_front.move_voltage(1000); //third ring


			//third ring



    }

    pros::delay(10);
  }
}