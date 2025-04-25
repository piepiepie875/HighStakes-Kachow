/* Things to do in code:
 * 1. Add the color sort
 * 6. Test out different drive curves
 */

 #include "main.h"
 #include "lemlib/api.hpp" // IWYU pragma: keep
 #include "pros/colors.h"
 #include "pros/llemu.hpp"
 #include "pros/misc.h"
 #include "pros/rotation.hpp"
 #include "pros/rtos.hpp"
 #include "pros/screen.h"
 #include "pros/screen.hpp"
 lv_obj_t* image;
 
 #define INTAKE_PORT -21
 #define LEFT_LADYBROWN_PORT -1
 #define RIGHT_LADYBROWN_PORT 12
 #define CLAMP_PORT 'G'
 #define LEFT_DOINKER_PORT 'H'
 #define RIGHT_DOINKER_PORT 'F' 
 
 
 pros::Rotation rotationSensor(-13);
 pros::adi::DigitalOut Clamp(CLAMP_PORT);
 pros::adi::DigitalOut LeftDoinker(LEFT_DOINKER_PORT);
 pros::adi::DigitalOut RightDoinker(RIGHT_DOINKER_PORT);
 pros::Motor Intake(INTAKE_PORT);
 pros::Motor LeftLadyBrown(LEFT_LADYBROWN_PORT);
 pros::Motor RightLadyBrown(RIGHT_LADYBROWN_PORT);
 pros::Optical colorSensor(19);
 pros::Controller master(pros::E_CONTROLLER_MASTER);
 
 // motor groups
 pros::MotorGroup leftMotors({-4, -5, -6}, pros::MotorGearset::blue);
 pros::MotorGroup rightMotors({8, 9, 10}, pros::MotorGearset::blue); 
 
 // Inertial Sensor
 pros::Imu imu(1);
 
 // tracking wheels
 // horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
 pros::Rotation horizontalEnc(-3);
 // vertical tracking wheel encoder. Rotation sensor, port 11, reversed
 // pros::Rotation verticalEnc(-11);
 // horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot
 // (negative)
 lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2,
                                  -0.75);
 // vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot
 // (negative)
 // lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275,
 // -2.5);
 
 // drivetrain settings
 lemlib::Drivetrain drivetrain(
     &leftMotors,                // left motor group
     &rightMotors,               // right motor group
     11.375,                     // track width
     lemlib::Omniwheel::NEW_325, // wheel diameter
     450,                        // drivetrain rpm
     2 // Horizontal Drift is 2 with all omni drive. If we had traction wheels, it would have been 8
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
                      0  // maximum acceleration (slew)
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
                             nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                             nullptr, // horizontal tracking wheel
                             nullptr,     // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                             &imu // inertial sensor
 );
 
 // Test out increasing Curve, decreasing min output, and decreasing deadband input curve for throttle input during driver control. Decrease minoutput first
 lemlib::ExpoDriveCurve
     throttleCurve(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
     );
 
 // try decreasing curve, decreasing deadband, and decreasing min output --> should help dhruva turn more accurately at low speeds. Decrease curve first
 lemlib::ExpoDriveCurve
     steerCurve(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.019 // expo curve gain
     );
 
 // create the chassis
 lemlib::Chassis chassis(drivetrain, linearController, angularController,
                         sensors, &throttleCurve, &steerCurve);
 
 // Lady Brown PID 
 double angles[] = {3, 28, 145, 118, 155};
 int curAngle = 0;
 double kP = 2;
 double kD = 3;       
 double deadband = 2; 
 double prevError = 0;
 
 void PIDcontrol() {
   while (true) {
     double target = angles[curAngle];
     double currentPos = rotationSensor.get_position() / 100.0;
     double error = target - currentPos;
 
     // Dynamically set kP
     if(curAngle == 1){
       kP = 2;
     }
     else if(curAngle == 2 || curAngle == 3){
       kP = 0.08;
     }
     else{
       kP = 0.9;
     }
 
     if (fabs(error) > 60 && (curAngle == 2 || curAngle == 3)) {
       int direction = (error > 0) ? 1 : -1;
       LeftLadyBrown.move_velocity(200 * direction);
       RightLadyBrown.move_velocity(200 * direction);
     }
     else if (fabs(error) > deadband) {
       double derivative = error - prevError;
       double vel = kP * error + kD * derivative;
 
       LeftLadyBrown.move_velocity(vel);
       RightLadyBrown.move_velocity(vel);
     }
     else {
       LeftLadyBrown.move_velocity(0);
       RightLadyBrown.move_velocity(0);
     }
 
     prevError = error;
     pros::delay(10);
   }
 }
 
 // Auton Selector + KACHOW Logo
 int autonomousMode = 1;
 
 const int buttonWidth = 95;
 const int buttonHeight = 45;
 const int cols = 4;
 const int rows = 3;
 const int xSpacing = 20;
 const int ySpacing = 20;
 const int startX = 10;
 const int startY = 10;
 
 const char *buttonLabels[12] = {
     "Blue AWP", "Red AWP", "Blue Neg",    "Red Neg",    "Blue Pos", "Red Pos",
     "Blue TN",  "Red TN",  "Blue TN Mid", "Red TN Mid", "Blue TP",  "Red TP"};
 
 void draw_buttons() {
   pros::screen::erase();
 
   for (int i = 0; i < 12; i++) {
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
           pros::screen::print(pros::E_TEXT_MEDIUM, 10, 210, "Selected: %s", buttonLabels[i]);
         }
       }
     }
     pros::delay(10);
   }
 }

//  extern "C" {
//    LV_IMG_DECLARE(Image); // Or whatever your image name is
//  }
 
// //  void draw_KACHOW() {
//      pros::screen::erase();
     
   
//     //  image = lv_img_create(lv_scr_act());
//     //  lv_img_set_src(image, &Image);
//     //  lv_obj_set_size(image, 480, 272);
//     //  lv_obj_align(image, LV_ALIGN_CENTER, 0, 0);
   
//  }
 
 bool colorSortOn = true;
 bool noStop = true;
 
 void colorSort() {
   while (true) {
     if (colorSortOn) {
       colorSensor.set_led_pwm(100);
       int hue = colorSensor.get_hue();
 
       if (autonomousMode % 2 == 1) { // If autonomousMode is odd, check for red
         if (hue < 50) {
           pros::delay(116); // CHANGE THESE VALUES
           noStop = false;
           Intake.move_velocity(-600); // Stop the motor
           pros::delay(150);        // CHANGE THESE VALUES
           noStop = true;
           Intake.move_velocity(
               600); // Restart the motor (adjust speed as needed)
         }
       } else { // If autonomousMode is even, check for blue
         if (hue > 170 && hue < 230) {
           pros::delay(116); // CHANGE THESE VALUES
           noStop = false;
           Intake.move_velocity(-600); // Stop the motor
           pros::delay(150);        // CHANGE THESE VALUES
           noStop = true;
           Intake.move_velocity(600); // Restart the motor (adjust speed as needed)
         }
       }
     }
 
     pros::delay(10); // Small delay to prevent CPU overload
   }
 }
 
 void initialize() {
   pros::lcd::initialize(); // initialize brain screen
  //  lv_init();
   chassis.calibrate();     // calibrate sensors
   imu.is_calibrating();
   rotationSensor.reset();
   rotationSensor.set_position(0);
   pros::Motor Intake(INTAKE_PORT);
   pros::Motor LeftLadyBrown(LEFT_LADYBROWN_PORT);
   pros::Motor RightLadyBrown(RIGHT_LADYBROWN_PORT);
   pros::adi::DigitalOut Clamp(CLAMP_PORT);
   pros::adi::DigitalOut LeftDoinker(LEFT_DOINKER_PORT);
   pros::adi::DigitalOut RightDoinker(RIGHT_DOINKER_PORT);
   pros::Controller master(pros::E_CONTROLLER_MASTER);
   pros::Task pidTask(PIDcontrol);
 
   // pros::Task colorCheckTask(colorSort);
 
   Intake.set_gearing(pros::E_MOTOR_GEAR_BLUE);
   Intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   LeftLadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
   LeftLadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   LeftLadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   RightLadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
   RightLadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   RightLadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 
  //  draw_buttons();
  //  pros::Task touchTask(check_touch);
    pros::Task screenTask([&]() {
      while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        pros::lcd::print(3, buttonLabels[autonomousMode-1]);
        pros::lcd::print(4, colorSortOn ? "Color Sort On" : "Color Sort Off");
        // pros::lcd::print(5, "kp: %f", kP);
        // pros::lcd::print(6, "kD: %f", kD);
        // pros::lcd::print(7, "Green: %f", colorSensor.get_rgb().green);
        // pros::lcd::print(4, "Value: %f", autonomousMode);


        // log position telemetry
        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);
      }
    });
 }
 
 /**
  * Runs while the robot is disabled
  */
 void disabled() {
   imu.is_calibrating();
   pros::Motor Intake(INTAKE_PORT);
   pros::Motor LeftLadyBrown(LEFT_LADYBROWN_PORT);
   pros::Motor RightLadyBrown(RIGHT_LADYBROWN_PORT);
   pros::adi::DigitalOut Clamp(CLAMP_PORT);
   pros::adi::DigitalOut LeftDoinker(LEFT_DOINKER_PORT);
   pros::adi::DigitalOut RightDoinker(RIGHT_DOINKER_PORT);
   pros::Controller master(pros::E_CONTROLLER_MASTER);
   pros::Task pidTask(PIDcontrol);
   // pros::Task colorCheckTask(colorSort);
 
   Intake.set_gearing(pros::E_MOTOR_GEAR_BLUE);
   Intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   LeftLadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
   LeftLadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   LeftLadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   RightLadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
   RightLadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   RightLadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 
   pros::Task touchTask(check_touch);
 }
 
 /**
  * runs after initialize if the robot is connected to field control
  */
 void competition_initialize() {
   imu.is_calibrating();
   rotationSensor.set_position(0);
   pros::Motor Intake(INTAKE_PORT);
   pros::Motor LeftLadyBrown(LEFT_LADYBROWN_PORT);
   pros::Motor RightLadyBrown(RIGHT_LADYBROWN_PORT);
   pros::adi::DigitalOut Clamp(CLAMP_PORT);
   pros::adi::DigitalOut LeftDoinker(LEFT_DOINKER_PORT);
   pros::adi::DigitalOut RightDoinker(RIGHT_DOINKER_PORT);
   pros::Controller master(pros::E_CONTROLLER_MASTER);
   pros::Task pidTask(PIDcontrol);
   // pros::Task colorCheckTask(colorSort);
 
   Intake.set_gearing(pros::E_MOTOR_GEAR_BLUE);
   Intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   LeftLadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
   LeftLadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   LeftLadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   RightLadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
   RightLadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   RightLadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 
   draw_buttons();
   pros::Task touchTask(check_touch);
 }
 
 // get a path used for pure pursuit
 // this needs to be put outside a function
 // ASSET(example_txt); // '.' replaced with "_" to make c++ happy
 
 void blueAWP() {
  chassis.setPose(85, 16, 212);
  chassis.moveToPoint(82.25, 9.75, 2000);
  pros::delay(100);
  curAngle = 3;
  pros::delay(500);
  chassis.moveToPoint(97.5, 45, 3000, {.forwards = false, .maxSpeed = 70});
  curAngle = 0;
  pros::delay(1200);
  Clamp.set_value(true);
  chassis.turnToPoint(110, 59, 700);
 Intake.move_velocity(600);
  chassis.moveToPoint(110, 59, 2000);
  chassis.turnToHeading(90, 700);
  chassis.moveToPoint(131, 55.5, 2500);
  chassis.moveToPoint(96, 48, 2500, {.forwards = false});
  // chassis.turnToHeading(90, 400);
  chassis.moveToPoint(119, 42, 2000);
  chassis.turnToHeading(220, 1200);
  // pros::delay(1200);
  chassis.moveToPoint(60, 27, 4000);
  pros::delay(800);
  Intake.move_velocity(-600);
  Clamp.set_value(false);
  chassis.turnToHeading(160, 700);
  Intake.move_velocity(0);
  chassis.moveToPoint(47, 54, 2000, {.forwards = false, .maxSpeed = 70});
  pros::delay(800);
  Clamp.set_value(true);
  chassis.turnToHeading(-90, 600);
  Intake.move_velocity(600);
  chassis.moveToPoint(23, 57, 2000);
  chassis.turnToHeading(90, 1000);
  chassis.moveToPoint(72, 52, 2000);
  Intake.move_velocity(0);


  // chassis.turnToHeading(207, 700);
  // chassis.moveToPoint(108, 24, 2000);
  // chassis.turnToHeading(270, 700);
  // chassis.moveToPoint(60, 24, 1000);
  // Clamp.set_value(false);
//   pros::delay(2500);
//   Intake.move_velocity(0);
//   chassis.turnToHeading(180, 200);
//   chassis.moveToPoint(48, 52, 2000, {.forwards = false});
//   Clamp.set_value(true);
  }
 
 void redAWP() {}
 
 void blueNegative() {
  chassis.setPose(85, 16, 212);
  chassis.moveToPoint(82.25, 9.5, 2000);
  pros::delay(100);
  curAngle = 3;
  pros::delay(500);
  chassis.moveToPoint(96, 45, 3000, {.forwards = false, .maxSpeed = 70});
  curAngle = 0;
  pros::delay(1000);
  Clamp.set_value(true);
  chassis.turnToPoint(110, 60, 700);
  Intake.move_velocity(600);
  chassis.moveToPoint(110, 60, 2000);
  chassis.turnToHeading(90, 700);
  chassis.moveToPoint(131, 56.5, 2500);
  chassis.moveToPoint(96, 48, 2500, {.forwards = false});
  // chassis.turnToHeading(90, 400);
  chassis.moveToPoint(119, 42, 2000);
  chassis.turnToHeading(180, 500);
  chassis.moveToPoint(117, 8, 2000);
  chassis.turnToPoint(127, -2, 500);
  Intake.move_velocity(0);
  chassis.moveToPoint(127, -2, 1200, {.maxSpeed = 50});
  pros::delay(400);
  Intake.move_velocity(600);
  pros::delay(800);
  chassis.moveToPoint(109, 16, 1000, {.forwards = false});
  // pros::delay(500);
  chassis.moveToPoint(121, 6, 600);
  chassis.moveToPoint(109, 16, 1000, {.forwards = false});
  pros::delay(200);
  chassis.turnToPoint(68, 36, 1000);
  chassis.moveToPoint(68, 36, 2000, {.minSpeed = 80});
 }
 
 void redNegative() {}

 void bluePositive() {
  chassis.setPose(59, 16, 148);
  chassis.moveToPoint(63.5, 10.6, 2000);
  pros::delay(100);
  curAngle = 3;
  pros::delay(500);
  chassis.moveToPoint(48, 43.5, 3000, {.forwards = false, .maxSpeed = 60});
  curAngle = 0;
  pros::delay(1200);
  Clamp.set_value(true);
  Intake.move_velocity(600);
  pros::delay(100);
  chassis.turnToHeading(-90, 1000);
  chassis.moveToPoint(20, 46, 2000);
  chassis.moveToPoint(22, 46, 2000, {.forwards = false});
  chassis.turnToPoint(14,10, 1000);
  // LeftDoinker.set_value(true);
  chassis.moveToPoint(14, 10, 2000);
  chassis.turnToPoint(7, 2, 500);
  Intake.move_velocity(0);
  chassis.moveToPoint(7, 2, 700, {.maxSpeed = 50});
  pros::delay(400);
  Intake.move_velocity(600);
  pros::delay(800);
  chassis.moveToPoint(30, 25, 1000, {.forwards = false});
  pros::delay(1000);
  chassis.moveToPoint(10, 5, 600);
  pros::delay(200);
  chassis.moveToPoint(25, 20, 1000, {.forwards = false});
  chassis.turnToPoint(90, 37, 1000);
  chassis.moveToPoint(90, 37, 2000, {.maxSpeed = 80});
  pros::delay(300);
  Clamp.set_value(false);
 }
 
 void redPositive() {}
 
 void blueTournamentNegative() {
  chassis.setPose(95, 19, 25.54);
  chassis.moveToPoint(111,60,3000);
  pros::delay(400);
  RightDoinker.set_value(true);
  Intake.move_velocity(600);
  chassis.turnToHeading(58, 400);
  pros::delay(50);
  Intake.move_velocity(0);
  chassis.moveToPoint(79, 47, 2000, {.forwards = false});
  pros::delay(600);
  Clamp.set_value(true);
  Intake.move_velocity(600);
  pros::delay(200);
  RightDoinker.set_value(false);
  chassis.moveToPoint(102, 48,2000);
  chassis.moveToPoint(122, 47,2000);
  chassis.moveToPoint(117, 47,2000, {.forwards = false});
  pros::delay(100);
  chassis.turnToHeading(175, 600);
  chassis.moveToPoint(115, 11,  2000);
  Intake.move_velocity(0);
  chassis.turnToHeading(135, 600);
  chassis.moveToPoint(121, 5, 1000);
  pros::delay(200);
  Intake.move_velocity(600);
  chassis.moveToPoint(107, 19,  2000, {.forwards = false});
  chassis.moveToPoint(116, 10, 2000);
  pros::delay(100);
  chassis.moveToPoint(109, 17,  2000, {.forwards = false});
  chassis.turnToHeading(-90, 1000);
  chassis.moveToPoint(76, 17, 2000);
 }
 
 void redTournamentNegative() {}
 
 void blueTournamentNegativeMiddle() {}
 
 void redTournamentNegativeMiddle() {}
 
 void blueTournamentPositive() {}
 
 void redTournamentPositive() {}
 
 void autonomous() {
  //  draw_KACHOW();
  //  chassis.setPose(0,0,0);
  //  chassis.moveToPose(-48, 36, -90, 4000, {.maxSpeed = 80});
   // chassis.turnToHeading(90, 000);
   switch (autonomousMode) {
   case 1:
     blueAWP();
     break;
   case 2:
     redAWP();
     break;
   case 3:
     blueNegative();
     break;
   case 4:
     redNegative();
     break;
   case 5:
     bluePositive();
     break;
   case 6:
     redPositive();
     break;
   case 7:
     blueTournamentNegative();
     break;
   case 8:
     redTournamentNegative();
     break;
   case 9:
     blueTournamentNegativeMiddle();
     break;
   case 10:
     redTournamentNegativeMiddle();
     break;
   case 11:
     blueTournamentPositive();
     break;
   case 12:
     redTournamentPositive();
     break;
   }
 }
 
 void opcontrol() {
   pros::Motor Intake(INTAKE_PORT);
   pros::Motor LeftLadyBrown(LEFT_LADYBROWN_PORT);
   pros::Motor RightLadyBrown(RIGHT_LADYBROWN_PORT);
   pros::adi::DigitalOut Clamp(CLAMP_PORT);
   pros::adi::DigitalOut LeftDoinker(LEFT_DOINKER_PORT);
   pros::adi::DigitalOut RightDoinker(RIGHT_DOINKER_PORT);
   pros::Controller master(pros::E_CONTROLLER_MASTER);
   pros::Task pidTask(PIDcontrol);
   // pros::Task colorCheckTask(colorSort);
 
   Intake.set_gearing(pros::E_MOTOR_GEAR_BLUE);
   Intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   LeftLadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
   LeftLadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   LeftLadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   RightLadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
   RightLadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
   RightLadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 
   // controller
   // loop to continuously update motors
  //  draw_KACHOW();
   while (true) {
     // get joystick positions
     int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
     int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
     // move the chassis with curvature drive
     chassis.tank(leftY, rightY);
 
     // intake
     if (master.get_digital(DIGITAL_R2) && noStop) {
       Intake.move_velocity(600);
     } else if (master.get_digital(DIGITAL_L2)) {
       Intake.move_velocity(-600);
     } else {
       Intake.move_velocity(0);
     }
 
     // clamp
     static bool toggle = false;
     if (master.get_digital_new_press(DIGITAL_R1)) {
       if (!toggle) {
         Clamp.set_value(true);
         toggle = !toggle;
       } else {
         Clamp.set_value(false);
         toggle = !toggle;
       }
     }
 
     // doinker
     static bool toggle2 = false;
     if (master.get_digital_new_press(DIGITAL_DOWN)) {
       if (!toggle2) {
         LeftDoinker.set_value(true);
         toggle2 = !toggle2;
       } else {
         LeftDoinker.set_value(false);
         toggle2 = !toggle2;
       }
     }
 
     static bool toggle3 = false;
     if (master.get_digital_new_press(DIGITAL_B)) {
       if (!toggle3) {
         RightDoinker.set_value(true);
         toggle3 = !toggle3;
       } else {
         RightDoinker.set_value(false);
         toggle3 = !toggle3;
       }
     }
 
     // lady brown
     if (master.get_digital_new_press(DIGITAL_L1)) {
       curAngle++;
       curAngle = curAngle % 3;
     }
 
     // color sort
     if (master.get_digital_new_press(DIGITAL_UP)) {
       curAngle = 3;
     }
 
     // delay to save resources
     pros::delay(10);
   }
 }