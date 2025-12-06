#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/colors.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-19, 20, -21}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({12, -13, 14}, pros::MotorGearset::blue);

// Intake Motors
pros::Motor Intake(-17, pros::MotorGearset::blue); // change
pros::Motor Hood(15, pros::MotorGearset::blue); // change

// Pneumatics
pros::adi::DigitalOut MatchLoader('A'); // change port letter
pros::adi::DigitalOut HoodPiston('B'); // change port letter

// Inertial Sensor
pros::Imu imu(18); //change

// tracking wheels
pros::Rotation horizontalEnc(-16); //change
// pros::Rotation verticalEnc(-11);

lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 4.375); // up is positive, back is negative
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5); // Left is negative, right is positive

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,                // left motor group
                              &rightMotors,               // right motor group
                              11.375,                     // track width
                              lemlib::Omniwheel::NEW_325, // wheel diameter
                              450,                        // drivetrain rpm
                              8 // Horizontal Drift is 2 with all omni drive. If
                                // we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            &imu     // inertial sensor
);

// Test out increasing Curve, decreasing min output, and decreasing deadband
// input curve for throttle input during driver control. Decrease minoutput
// first
lemlib::ExpoDriveCurve
    throttleCurve(3,    // joystick deadband out of 127
                  10,   // minimum output where drivetrain will move out of 127
                  1.019 // expo curve gain
    );

// try decreasing curve, decreasing deadband, and decreasing min output -->
// should help dhruva turn more accurately at low speeds. Decrease curve first
lemlib::ExpoDriveCurve
    steerCurve(3,    // joystick deadband out of 127
               10,   // minimum output where drivetrain will move out of 127
               1.019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);

// Auton Selector
int autonomousMode = 0;
int matchLoaderState = 0; // 0 is retracted, 1 is extended

const int buttonWidth = 100;
const int buttonHeight = 30;
const int cols = 3;
const int rows = 2;
const int xSpacing = 50;
const int ySpacing = 20;
const int startX = 10;
const int startY = 100;

const char *buttonLabels[6] = {
    "AWP", "Right Side", "Left Side", "Comp Right", "Comp Left", "Skills"};

void draw_buttons() {
  pros::screen::erase();

  for (int i = 0; i < 6; i++) {
    pros::screen::set_pen(pros::c::COLOR_RED);
    int col = i % cols;
    int row = i / cols;
    int x = startX + col * (buttonWidth + xSpacing);
    int y = startY + row * (buttonHeight + ySpacing);

    // Draw button
    pros::screen::fill_rect(x, y, x + buttonWidth, y + buttonHeight);

    // Print label
    pros::screen::set_pen(pros::c::COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, x + 5, y + 15, buttonLabels[i]);
  }
}

void check_touch() {
  while (true) {
    pros::screen_touch_status_s_t touch = pros::screen::touch_status();

    if (touch.touch_status == TOUCH_PRESSED) {
      int tx = touch.x;
      int ty = touch.y;

      for (int i = 0; i < 12; i++) {
        int col = i % cols;
        int row = i / cols;
        int x = startX + col * (buttonWidth + xSpacing);
        int y = startY + row * (buttonHeight + ySpacing);

        if (tx >= x && tx <= x + buttonWidth && ty >= y &&
            ty <= y + buttonHeight) {
          autonomousMode = i + 1;
          pros::screen::set_pen(pros::c::COLOR_BLACK);
          pros::screen::fill_rect(0, 200, 480, 240); // Clear bottom
          pros::screen::set_pen(pros::c::COLOR_YELLOW);
          pros::screen::print(pros::E_TEXT_MEDIUM, 10, 210, "Selected: %s",
                              buttonLabels[i]);
        }
      }
    }
    pros::delay(10);
  }
}

void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  imu.reset();
  MatchLoader.set_value(false); // Initialize piston to retracted state
  HoodPiston.set_value(false); // Initialize piston to retracted state
  chassis.setBrakeMode(MOTOR_BRAKE_COAST);
  Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  Hood.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  

   draw_buttons();
   pros::Task touchTask(check_touch);
  pros::Task screenTask([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

      // log position telemetry"
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // delay to save resources
      pros::delay(50);
    }
  });
}

// reset motors
void disabled() {
  
}

// initalize everything
void competition_initialize() {
  draw_buttons();
  pros::Task touchTask(check_touch);
}

void intakeHold(){
  Intake.move_velocity(600);
  Hood.move_velocity(-600);
  HoodPiston.set_value(false);
}

void intakeScore(){
  HoodPiston.set_value(true);
  Intake.move_velocity(600);
  Hood.move_velocity(600);
}

void intakeStop(){
  Intake.move_velocity(0);
  Hood.move_velocity(0);
}

void outake(){
  Intake.move_velocity(-600);
  Hood.move_velocity(-600);
}

void AWP(){

}

void rightSide(){

}

void leftSide(){

}

void compRight(){

}

void compLeft(){

}

void skills1(){
  chassis.setPose(55.75, 24.75, -90); // reset pose at start of auton
  chassis.moveToPoint(22,24.75, 5000);
  chassis.turnToHeading(180, 800);
  MatchLoader.set_value(true);
  pros::delay(100);
  intakeHold();
  chassis.moveToPoint(24, 0, 4000, {.maxSpeed = 60});
  chassis.waitUntilDone();
  intakeStop();
  chassis.setPose(24,15, chassis.getPose().theta);
  chassis.moveToPoint(25.25, 44, 3000, {.forwards = false, .maxSpeed = 80});
  pros::delay(400);
  MatchLoader.set_value(false);
  chassis.moveToPoint(25.25, 46, 3000, {.forwards = false});
  pros::delay(300);
  intakeScore();
  pros::delay(6000);
  chassis.moveToPose(48,7.5,90, 6000);
  pros::delay(3000);
  MatchLoader.set_value(true);
  outake();
  pros::delay(1000);
  chassis.moveToPoint(72, 7.5, 5000);
  pros::delay(4000);
  MatchLoader.set_value(false);
  intakeStop();
}

void skills2(){
  chassis.setPose(55.75, 24.75, -90); // reset pose at start of auton
  chassis.moveToPoint(22,24.75, 5000);
  chassis.turnToHeading(180, 800);
  MatchLoader.set_value(true);
  pros::delay(100);
  intakeHold();
  chassis.moveToPoint(24, 0, 4000, {.maxSpeed = 60});
  chassis.waitUntilDone();
  intakeStop();
  chassis.setPose(24,15, chassis.getPose().theta);
  chassis.moveToPoint(25.25, 44, 3000, {.forwards = false, .maxSpeed = 80});
  pros::delay(400);
  MatchLoader.set_value(false);
  chassis.moveToPoint(25.25, 46, 3000, {.forwards = false});
  intakeScore();
  pros::delay(6000);
  chassis.moveToPose(120, 24, 90, 8000);

}

// switch statement for autos
void autonomous() {
  switch (autonomousMode) {
  case 1:
    AWP();
    break;
  case 2:
    rightSide();
    break;
  case 3:
    leftSide();
    break;
  case 4:
    compRight();
    break;
  case 5:
    compLeft();
    break;
  case 6:
    skills1();
    break;
  default:
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000);
  }
}

void opcontrol() {
  while (true) {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    chassis.tank(leftY, rightY);

    // delay to save resources
    pros::delay(10);

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      HoodPiston.set_value(false);
      Intake.move_velocity(600);
      Hood.move_velocity(-600);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      HoodPiston.set_value(true);
      Intake.move_velocity(600);
      Hood.move_velocity(600);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      Intake.move_velocity(-600);
      Hood.move_velocity(-600);
    }
    else{
      Intake.move_velocity(0);
      Hood.move_velocity(0);
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
      MatchLoader.set_value(!matchLoaderState);
      matchLoaderState = !matchLoaderState;
    }

    //for later, left and right back things are for horns
  }
}