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
pros::MotorGroup leftMotors({-4, -5, -6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({8, 9, 10}, pros::MotorGearset::blue);

// Inertial Sensor
pros::Imu imu(1);

// tracking wheels
pros::Rotation horizontalEnc(-3);
// pros::Rotation verticalEnc(-11);

lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.75); // up is positive, back is negative
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5); // Left is negative, right is positive

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,                // left motor group
                              &rightMotors,               // right motor group
                              11.375,                     // track width
                              lemlib::Omniwheel::NEW_325, // wheel diameter
                              450,                        // drivetrain rpm
                              2 // Horizontal Drift is 2 with all omni drive. If
                                // we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings
    linearController(8,   // proportional gain (kP)
                     0,   // integral gain (kI)
                     9,   // derivative gain (kD)
                     3,   // anti windup
                     1,   // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     0    // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(10, // proportional gain (kP)
                      0,  // integral gain (kI)
                      3,  // derivative gain (kD)
                      0,  // anti windup
                      0,  // small error range, in inches
                      0,  // small error range timeout, in milliseconds
                      0,  // large error range, in inches
                      0,  // large error range timeout, in milliseconds
                      0   // maximum acceleration (slew)
    );

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
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
int autonomousMode = 1;

const int buttonWidth = 95;
const int buttonHeight = 45;
const int cols = 4;
const int rows = 3;
const int xSpacing = 20;
const int ySpacing = 20;
const int startX = 10;
const int startY = 10;

const char *buttonLabels[5] = {
    "AWP", "Right Side", "Left Side", "Comp Right", "Comp Left"};

void draw_buttons() {
  pros::screen::erase();

  for (int i = 0; i < 5; i++) {
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

//    draw_buttons();
//    pros::Task touchTask(check_touch);
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

// switch statement for autos
void autonomous() {

}

void opcontrol() {
  while (true) {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    chassis.tank(leftY, rightY);

    // delay to save resources
    pros::delay(10);
  }
}