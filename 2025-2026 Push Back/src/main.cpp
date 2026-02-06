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
#include <cmath>

pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-13, 15, -16}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({17, -18, 19}, pros::MotorGearset::blue);

// Intake Motors
pros::MotorGroup Intake({11,-20}, pros::MotorGearset::blue);

// Pneumatics
pros::adi::DigitalOut MatchLoader('D');
pros::adi::DigitalOut Holder('A');
pros::adi::DigitalOut MiddleGoal('E');
pros::adi::DigitalOut LeftWing('B');
pros::adi::DigitalOut RightWing('C');

// Inertial Sensor
pros::Imu imu(10); //change

// tracking wheels
pros::Rotation horizontalEnc(21); //change
// pros::Rotation verticalEnc(-11);

lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -1.5); // up is positive, back is negative
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
lemlib::ControllerSettings linearController(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
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
int leftWingState = 0; // 0 is retracted, 1 is extended
int rightWingState = 0; // 0 is retracted, 1 is extended

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
  Holder.set_value(false); // Initialize piston to retracted state
  chassis.setBrakeMode(MOTOR_BRAKE_COAST);
  Intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  

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
  Holder.set_value(false);
  MiddleGoal.set_value(false);
  Intake.move_velocity(600);
}

void intakeScoreHigh(){
  Holder.set_value(true);
  MiddleGoal.set_value(false);
  Intake.move_velocity(600);
}

void intakeScoreMiddle(){
  Holder.set_value(false);
  MiddleGoal.set_value(true);
  Intake.move_velocity(600);
}

void intakeStop(){
  Intake.move_velocity(0);
}

void outake(){
  Intake.move_velocity(-600);
}

void AWP(){
  chassis.setPose(88.25, 24.75, -90); // reset pose at start of auton (flipped: 144 - 55.75 = 88.25)
  chassis.moveToPoint(120.5, 24.75, 2000); // flipped: 144 - 23.5 = 120.5
  chassis.turnToHeading(180, 800);
  MatchLoader.set_value(true);
  intakeHold();
  chassis.moveToPoint(120, -20, 1500, {.maxSpeed = 60}); // flipped: 144 - 24 = 120
  chassis.waitUntilDone();
  chassis.setPose(120, 14.5, chassis.getPose().theta); // flipped: 144 - 24 = 120
  chassis.moveToPoint(119.5, 50, 2200, {.forwards = false, .maxSpeed = 70, .earlyExitRange = 4}); // flipped: 144 - 24.5 = 119.5
  pros::delay(300);
  MatchLoader.set_value(false);
  pros::delay(200);
  intakeScoreHigh();
  chassis.waitUntilDone();
  chassis.setPose(120, 43.5, chassis.getPose().theta); // flipped: 144 - 24 = 120
  chassis.moveToPoint(120, 36, 1000); // flipped: 144 - 24 = 120
  chassis.moveToPose(91, 53, 135, 4000, {.minSpeed = 50}); // flipped: 144 - 53 = 91, angle: 180 - 45 = 135
  intakeHold();
  chassis.waitUntilDone();
  chassis.turnToHeading(-45, 1100); // flipped: 180 - (-135) = 45, negated = -45
  chassis.moveToPoint(81, 61, 2000, {.forwards = false}); // flipped: 144 - 63 = 81
  pros::delay(500);
  intakeScoreMiddle();
  pros::delay(500);
  chassis.moveToPoint(93, 51, 2000, {.minSpeed = 50}); // flipped: 144 - 51 = 93
  chassis.turnToHeading(-90, 800); // flipped: 180 - 90 = 90, negated = -90
  chassis.moveToPoint(46, 51, 2000, {.minSpeed = 60}); // flipped: 144 - 98 = 46
  intakeHold();
  chassis.turnToHeading(-45, 800); // flipped: 180 - 135 = 45, negated = -45
  chassis.moveToPoint(18, 24, 3000, {.minSpeed = 50}); // flipped: 144 - 126 = 18
  chassis.turnToHeading(180, 600);
  // MatchLoader.set_value(true);
  // pros::delay(1000);
  // intakeHold();  
  // chassis.moveToPoint(120, -20, 1600, {.maxSpeed=60});
  // chassis.waitUntilDone();
  // intakeStop();
  // chassis.setPose(120,14.5, chassis.getPose().theta);
  chassis.moveToPoint(18, 50, 3000, {.forwards = false, .earlyExitRange = 4}); // flipped: 144 - 126 = 18
  // pros::delay(300);
  // MatchLoader.set_value(false);
  pros::delay(600);
  intakeScoreHigh();
}

void rightSide(){
  chassis.setPose(79, 25, 0); // mirrored from (65, 25, 0)
  intakeHold();
  chassis.moveToPoint(96, 48, 3000); // mirrored from (45, 48)
  pros::delay(500);
  intakeStop();
  chassis.turnToHeading(-45, 1000); // mirrored from -125
  chassis.moveToPoint(92, 55.25, 2000); // mirrored from (57.5, 54.75)
  pros::delay(800);
  outake();
  pros::delay(100);
  intakeStop();
  // chassis.moveToPoint(119.5, 24, 4000); // mirrored from (27.5, 24)
  // pros::delay(200);
  // intakeHold();
  // chassis.turnToHeading(180, 1000);
  // MatchLoader.set_value(true);
  // pros::delay(1000);
  // intakeHold();
  // chassis.moveToPoint(120.25, 0, 1500, {.maxSpeed=60}); // mirrored from (26.75, 0)
  // chassis.waitUntilDone();
  // intakeStop();
  // chassis.setPose(120, 15, chassis.getPose().theta); // mirrored from (24, 15)
  // chassis.moveToPoint(120, 45, 3000, {.forwards = false, .maxSpeed = 80}); // mirrored from (25, 45)
  // pros::delay(400);
  // MatchLoader.set_value(false);
  // chassis.moveToPoint(120, 47, 3000, {.forwards = false}); // mirrored from (25, 47)
  // pros::delay(300);
  // outake();
  // pros::delay(200);
  // intakeScoreHigh();
  // pros::delay(3000);
  // intakeStop();
  // pros::delay(100);
  // chassis.moveToPoint(119, 35, 3000); // mirrored from (25, 35)
  // Holder.set_value(false);
  // chassis.moveToPoint(120, 47, 3000, {.forwards = false, .maxSpeed = 60}); // mirrored from (25, 47)
  // pros::delay(200);
  // chassis.moveToPoint(119, 24, 3000); // mirrored from (27.5, 24)
}

void leftSide(){
  chassis.setPose(65, 25, 0);
  intakeHold();
  chassis.moveToPoint(45, 48, 3000);
  pros::delay(500);
  chassis.turnToHeading(-125, 1000);
  intakeStop();
  chassis.moveToPoint(57.5,54.75, 2000, {.forwards = false});
  pros::delay(800);
  intakeScoreMiddle();
  pros::delay(300);
  intakeStop();
  chassis.moveToPoint(27.5, 24, 4000);
  pros::delay(200);
  intakeHold();
  chassis.turnToHeading(180, 1000);
  MatchLoader.set_value(true);
  pros::delay(1000);
  intakeHold();
  chassis.moveToPoint(26.75, 0, 1500, {.maxSpeed=60});
  chassis.waitUntilDone();
  intakeStop();
  chassis.setPose(24,15, chassis.getPose().theta);
  chassis.moveToPoint(25, 45, 3000, {.forwards = false, .maxSpeed = 80});
  pros::delay(400);
  MatchLoader.set_value(false);
  chassis.moveToPoint(25, 47, 3000, {.forwards = false});
  pros::delay(300);
  outake();
  pros::delay(200);
  intakeScoreHigh();
  pros::delay(3000);
  intakeStop();
  pros::delay(100);
  chassis.moveToPoint(25, 35, 3000);
  Holder.set_value(false);
  chassis.moveToPoint(25, 47, 3000, {.forwards = false, .maxSpeed = 60});
  pros::delay(200);
  chassis.moveToPoint(25, 35, 3000);
}

void compRight(){
  chassis.setPose(79, 25, 0); // mirrored from (65, 25, 0)
  intakeHold();
  chassis.moveToPoint(97, 48, 3000); // mirrored from (45, 48)
  pros::delay(300);
  chassis.turnToPoint(114, 61.75, 1000); // mirrored from -125
  chassis.moveToPoint(114, 61.75, 2000); // mirrored from (57.5, 54.75)
  pros::delay(200);
  chassis.moveToPoint(97,48, 3000, {.forwards=false});
  chassis.moveToPoint(119.5, 26, 4000); // mirrored from (27.5, 24)
  pros::delay(200);
  chassis.turnToHeading(180, 1000);
  MatchLoader.set_value(true);
  pros::delay(1000);
  intakeHold();
  chassis.moveToPoint(120.25, 0, 2000, {.maxSpeed=60}); // mirrored from (26.75, 0)
  chassis.waitUntilDone();
  intakeStop();
  chassis.setPose(120, 15, chassis.getPose().theta); // mirrored from (24, 15)
  chassis.moveToPoint(120, 45, 3000, {.forwards = false, .maxSpeed = 80}); // mirrored from (25, 45)
  pros::delay(400);
  MatchLoader.set_value(false);
  chassis.moveToPoint(120, 47, 3000, {.forwards = false}); // mirrored from (25, 47)
  pros::delay(300);
  outake();
  pros::delay(200);
  intakeScoreHigh();
  pros::delay(3500);
  intakeStop();
  pros::delay(100);
  chassis.moveToPoint(119, 35, 3000); // mirrored from (25, 35)
  Holder.set_value(false);
  chassis.moveToPoint(120, 47, 3000, {.forwards = false, .maxSpeed = 60}); // mirrored from (25, 47)
  pros::delay(200);
  chassis.moveToPoint(119, 24, 3000); // mirrored from (27.5, 24)
}

void compLeft(){

}

void skills(){
  chassis.setPose(55.75, 24.75, -90); // reset pose at start of auton
  chassis.moveToPoint(23.5,24.75, 2000);
  chassis.turnToHeading(180, 1100);
  MatchLoader.set_value(true);
  intakeHold();
  chassis.moveToPoint(24, -20, 3000, {.maxSpeed = 60});
  chassis.waitUntilDone();
  chassis.setPose(24,14.5, chassis.getPose().theta);
  chassis.moveToPose(14, 36, 180, 2200, {.forwards = false});
  pros::delay(300);
  MatchLoader.set_value(false);
  chassis.moveToPose(14, 96, 180, 4000, {.forwards = false});
  chassis.swingToPoint(24, 104, DriveSide::RIGHT, 2500, {.forwards = false});
  pros::delay(700);
  intakeScoreHigh();
  pros::delay(1200);
  MatchLoader.set_value(true);
  chassis.moveToPoint(24, 164, 1500, {.maxSpeed = 60});
  chassis.waitUntilDone();
  chassis.setPose(24, 129.5 , chassis.getPose().theta); 
  chassis.moveToPoint(24.5, 94, 2200, {.forwards = false, .maxSpeed = 70, .earlyExitRange = 4});
  pros::delay(300);
  MatchLoader.set_value(false);
  pros::delay(200);
  intakeScoreHigh();
  chassis.waitUntilDone();
  chassis.setPose(24, 100.5, chassis.getPose().theta);
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
    skills();
    break;
  default:
    AWP();
  }
}

void opcontrol() {
  if(autonomousMode != 0){
    RightWing.set_value(true);
    LeftWing.set_value(true);
    rightWingState = 1;
    leftWingState = 1;
  }
  while (true) {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    chassis.tank(leftY, rightY);

    // delay to save resources
    pros::delay(10);

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      Holder.set_value(false);
      MiddleGoal.set_value(false);
      Intake.move_velocity(600);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      Holder.set_value(true);
      MiddleGoal.set_value(false);
      Intake.move_velocity(600);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      Holder.set_value(false);
      MiddleGoal.set_value(true);
      Intake.move_velocity(600);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      Intake.move_velocity(-600);
    }
    else{
      Intake.move_velocity(0);
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
      MatchLoader.set_value(!matchLoaderState);
      matchLoaderState = !matchLoaderState;
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
      LeftWing.set_value(!leftWingState);
      leftWingState = !leftWingState;
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
      RightWing.set_value(!rightWingState);
      rightWingState = !rightWingState;
    }
  }
}