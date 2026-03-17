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
                  5,   // minimum output where drivetrain will move out of 127
                  1.02 // expo curve gain
    );

// try decreasing curve, decreasing deadband, and decreasing min output -->
// should help dhruva turn more accurately at low speeds. Decrease curve first
lemlib::ExpoDriveCurve
    steerCurve(3,    // joystick deadband out of 127
               5,   // minimum output where drivetrain will move out of 127
               1.02 // expo curve gain
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
    "AWP", "Right Side", "Left Side", "Right 9+0", "Left 4+3 Wing", "Skills"};

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
  MatchLoader.set_value(false);
  LeftWing.set_value(false);
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
  chassis.setPose(88.25, 24.75, 90); 
  chassis.moveToPoint(120.2, 24.75, 1500, {.minSpeed = 60, .earlyExitRange = 9});
  chassis.turnToHeading(180, 800, {.minSpeed = 50, .earlyExitRange = 10});
  LeftWing.set_value(true);
  MatchLoader.set_value(true);
  intakeHold();
  chassis.moveToPoint(120.8, -20, 1400, {.maxSpeed = 80}); //first matchloader
  chassis.waitUntilDone();
  chassis.setPose(119, 14.5, chassis.getPose().theta); 
  chassis.moveToPoint(118.5, 50, 3000, {.forwards = false, .maxSpeed = 90, .earlyExitRange = 4}); //score in first long goal
  pros::delay(450);
  intakeScoreHigh();
  chassis.waitUntilDone();
  MatchLoader.set_value(false);
  chassis.setPose(120, 43.5, chassis.getPose().theta);
  chassis.moveToPoint(120, 32, 2000, {.minSpeed = 60, .earlyExitRange = 4});
  pros::delay(100);
  outake();
  chassis.turnToHeading(-40, 1500, {.minSpeed = 30, .earlyExitRange = 110});
  intakeHold();
  chassis.moveToPoint(92, 46, 2000, {.minSpeed = 60, .earlyExitRange = 4});  //pick up first set of balls
  pros::delay(800);
  MatchLoader.set_value(true);
  pros::delay(100);
  MatchLoader.set_value(false);
  chassis.turnToHeading(-90, 800, {.minSpeed = 30, .earlyExitRange = 10});
  chassis.moveToPoint(48, 45, 2500, {.minSpeed = 60, .earlyExitRange = 4}); //move to second set of balls
  pros::delay(800);
  // MatchLoader.set_value(true);
  chassis.turnToHeading(-135, 1000, {.minSpeed = 30, .earlyExitRange = 80});
  chassis.moveToPoint(59, 55, 1500, {.forwards = false}); //score in middle goal
  pros::delay(750);
  intakeScoreMiddle();
  MatchLoader.set_value(false);
  pros::delay(2000);
  chassis.moveToPoint(26, 24, 2000, {.minSpeed = 60, .earlyExitRange = 4});
  pros::delay(500);
  MatchLoader.set_value(true);
  chassis.turnToHeading(180, 800, {.minSpeed = 50, .earlyExitRange = 10});
  intakeHold();
  chassis.moveToPoint(25.2, -20, 1600, {.maxSpeed = 80}); //second matchloader
  chassis.waitUntilDone();
  chassis.setPose(25, 14.5, chassis.getPose().theta); 
  chassis.moveToPoint(25.5, 50, 1800, {.forwards = false, .maxSpeed = 90, .earlyExitRange = 4});  //score in second long goal
  pros::delay(300);
  MatchLoader.set_value(false);
  pros::delay(350);
  LeftWing.set_value(false);
  intakeScoreHigh();
  chassis.waitUntilDone();
}

void rightSide(){
  // prob wont even have it
  chassis.setPose(0,0,90);
  LeftWing.set_value(true);
  chassis.moveToPoint(4, 0, 2000);
  pros::delay(300);
  LeftWing.set_value(false);
}

void leftSide(){
  // what we doing here
  chassis.setPose(0,0,-90);
  LeftWing.set_value(true);
  chassis.moveToPoint(-4, 0, 2000);
  pros::delay(300);
  LeftWing.set_value(false);
}

void left4plus3(){
  chassis.setPose(65, 25, 0);
  LeftWing.set_value(true);
  chassis.moveToPoint(48, 48, 2000, {.minSpeed = 60, .earlyExitRange = 1});
  intakeHold();
  chassis.turnToHeading(-135, 1000, {.minSpeed = 50, .earlyExitRange = 80});
  chassis.moveToPoint(61.5, 58.5, 1500, {.forwards = false}); //score in middle goal
  pros::delay(800);
  intakeScoreMiddle();
  pros::delay(1500);
  chassis.moveToPoint(26, 24, 2000, {.minSpeed = 60, .earlyExitRange = 6});
  chassis.turnToHeading(180, 800, {.minSpeed = 50, .earlyExitRange = 20});
  MatchLoader.set_value(true);
  intakeHold();
  chassis.moveToPoint(26.2, -20, 1300, {.maxSpeed = 80}); //second matchloader
  chassis.waitUntilDone();
  chassis.setPose(25.5, 14.5, chassis.getPose().theta); 
  chassis.moveToPoint(27, 50, 1800, {.forwards = false, .maxSpeed = 90, .earlyExitRange = 4});  //score in second long goal
  pros::delay(300);
  MatchLoader.set_value(false);
  pros::delay(350);
  intakeScoreHigh();
  chassis.waitUntilDone();
  chassis.setPose(24, 43.5, chassis.getPose().theta);
  chassis.moveToPose(13, 28, 210, 3000, {.minSpeed = 60, .earlyExitRange = 4});
  chassis.turnToHeading(180, 800, {.minSpeed = 50, .earlyExitRange = 20});
  LeftWing.set_value(false);
  intakeStop();
  chassis.moveToPoint(15, 63, 2000, {.forwards = false, .maxSpeed = 70});
  chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
}

void right7ball(){
  chassis.setPose(79, 25, 0);
  LeftWing.set_value(true);
  chassis.moveToPoint(101, 50.5, 2000, {.minSpeed = 60, .earlyExitRange = 1});
  intakeHold();
  chassis.turnToHeading(135, 1000, {.minSpeed = 50, .earlyExitRange = 10});
  chassis.moveToPoint(124, 24, 2000, {.minSpeed = 60, .earlyExitRange = 6});
  pros::delay(400);
  MatchLoader.set_value(true);
  chassis.turnToHeading(180, 800, {.minSpeed = 50, .earlyExitRange = 20});
  intakeHold();
  chassis.moveToPoint(123.8, -20, 1100, {.maxSpeed = 80}); //second matchloader
  chassis.waitUntilDone();
  chassis.setPose(119, 14.5, chassis.getPose().theta);
  chassis.moveToPoint(118.5, 50, 2700, {.forwards = false, .maxSpeed = 90, .earlyExitRange = 4}); //score in second long goal
  pros::delay(650);
  intakeScoreHigh();
  chassis.waitUntilDone();
  MatchLoader.set_value(false);
  chassis.setPose(120, 43.5, chassis.getPose().theta);
  chassis.moveToPose(112, 30, 150, 3500, {.minSpeed = 60, .earlyExitRange = 4});
  chassis.turnToHeading(180, 800, {.minSpeed = 50, .earlyExitRange = 20});
  LeftWing.set_value(false);
  intakeStop();
  chassis.moveToPoint(113, 62, 2000, {.forwards = false, .maxSpeed = 70});
  chassis.setBrakeMode(MOTOR_BRAKE_HOLD);
}

void skills(){
  chassis.setPose(55.75, 24.75, -90); // reset pose at start of auton (flipped: 144 - 55.75 = 88.25)
  chassis.moveToPoint(25, 24.75, 1500); // flipped: 144 - 23.5 = 120.5
  chassis.turnToHeading(180, 800);
  MatchLoader.set_value(true);
  intakeHold();
  chassis.moveToPoint(24.5, -20, 2500, {.maxSpeed = 60}); // flipped: 144 - 24 = 120
  chassis.waitUntilDone();
  chassis.setPose(24,14.5, chassis.getPose().theta);
  chassis.moveToPoint(24, 22, 1500, {.forwards = false});
  LeftWing.set_value(true);
  chassis.moveToPose(14.5, 36, 180, 2200, {.forwards = false});
  pros::delay(600);
  MatchLoader.set_value(false);
  intakeStop();
  chassis.moveToPoint(14.5, 96, 4000, {.forwards = false, .maxSpeed = 70}); // hits the wall here
  chassis.moveToPose(26.5, 120, 240, 2000, {.forwards = false});
  chassis.waitUntilDone();
  chassis.turnToHeading(0, 900);
  chassis.moveToPoint(25.5, 80, 1000, {.forwards = false, .maxSpeed = 70});
  chassis.waitUntilDone();
  chassis.setPose(28, 102, chassis.getPose().theta);
  intakeScoreHigh();
  pros::delay(4500);
  MatchLoader.set_value(true);
  intakeHold();
  chassis.moveToPoint(28, 164, 3500, {.maxSpeed = 60});
  chassis.waitUntilDone();
  chassis.setPose(28, 129.5 , chassis.getPose().theta); 
  chassis.moveToPoint(28, 94, 2200, {.forwards = false, .maxSpeed = 70, .earlyExitRange = 4});
  pros::delay(600);
  MatchLoader.set_value(false);
  intakeScoreHigh();
  pros::delay(4000);
  chassis.waitUntilDone();
  chassis.setPose(24, 100.5, chassis.getPose().theta);
  chassis.moveToPoint(24, 110, 1500);
  intakeStop();
  chassis.turnToHeading(90, 800);
  chassis.moveToPoint(119, 115, 4000, {.maxSpeed = 70}); 
  chassis.turnToHeading(0, 800);
  MatchLoader.set_value(true);
  intakeHold();
  chassis.moveToPoint(119, 164, 3500, {.maxSpeed = 60}); //goes into matchloader 3

  // Flipped copy of recent steps (x -> 144 - x, y -> 144 - y)
  chassis.waitUntilDone();
  chassis.setPose(120, 129.5, chassis.getPose().theta);
  chassis.moveToPoint(120, 122, 1500, {.forwards = false});
  chassis.moveToPose(132.5, 108, 0, 2200, {.forwards = false});
  pros::delay(600);
  MatchLoader.set_value(false);
  intakeStop();
  chassis.moveToPose(132.5, 46, 0, 4000, {.forwards = false});
  chassis.moveToPose(120.5, 25, 60, 2000, {.forwards = false});
  chassis.waitUntilDone();
  chassis.turnToHeading(180, 900);
  chassis.moveToPoint(119.5, 64, 1000, {.forwards = false});
  chassis.waitUntilDone();
  chassis.setPose(114, 42, chassis.getPose().theta);
  intakeScoreHigh();
  pros::delay(4500);
  MatchLoader.set_value(true);
  intakeHold();
  chassis.moveToPoint(114, -20, 3500, {.maxSpeed = 60});
  chassis.waitUntilDone();
  chassis.setPose(116, 14.5, chassis.getPose().theta);
  chassis.moveToPoint(116, 50, 3200, {.forwards = false, .maxSpeed = 70, .earlyExitRange = 4});
  pros::delay(600);
  MatchLoader.set_value(false);
  intakeScoreHigh();
  chassis.waitUntilDone();
  chassis.setPose(120, 43.5, chassis.getPose().theta);
  chassis.moveToPoint(120, 20, 2000);
  chassis.turnToHeading(70, 1200);
  chassis.moveToPoint(60, 4, 4000, {.forwards = false, .minSpeed = 127});
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
    right7ball();
    break;
  case 5:
    left4plus3();
    break;
  case 6:
    skills();
    break;
  default:
    right7ball();
    break;
  }
}

void opcontrol() {
  // if(autonomousMode == 0){
  //   LeftWing.set_value(true);
  //   leftWingState = 1;
  // }
  while (true) {
    chassis.setBrakeMode(MOTOR_BRAKE_COAST);
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    chassis.tank(leftY, rightY);

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
      LeftWing.set_value(!leftWingState);
      leftWingState = !leftWingState;
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
      skills();
    }

    // delay to save resources
    pros::delay(10);
  }
}