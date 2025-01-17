#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-8, -9, -10},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(4);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
//pros::Rotation horizontalEnc(nullptr);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
//pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.375, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10.3, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            70 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.71, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             11.8, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
#define INTAKE_PORT 21
#define LADYBROWN_PORT -6
#define LADYBROWN2_PORT 5
#define CLAMP 'B'
#define GREENFN 'A'

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    pros::Motor Intake(INTAKE_PORT);
	Intake.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	Intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor LadyBrown(LADYBROWN_PORT);
	LadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
	LadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor LadyBrown2(LADYBROWN2_PORT);
	LadyBrown2.set_gearing(pros::E_MOTOR_GEAR_GREEN);
	LadyBrown2.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::adi::DigitalOut Clamp(CLAMP);
	pros::adi::DigitalOut GreenFN(GREENFN);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	LadyBrown2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	

    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
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
	chassis.calibrate();
	pros::Motor Intake(INTAKE_PORT);
	Intake.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	Intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor LadyBrown(LADYBROWN_PORT);
	LadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
	LadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor LadyBrown2(LADYBROWN2_PORT);
	LadyBrown2.set_gearing(pros::E_MOTOR_GEAR_GREEN);
	LadyBrown2.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::adi::DigitalOut Clamp(CLAMP);
	pros::adi::DigitalOut GreenFN(GREENFN);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	LadyBrown2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {
	chassis.calibrate();
	pros::Motor Intake(INTAKE_PORT);
	Intake.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	Intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor LadyBrown(LADYBROWN_PORT);
	LadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
	LadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor LadyBrown2(LADYBROWN2_PORT);
	LadyBrown2.set_gearing(pros::E_MOTOR_GEAR_GREEN);
	LadyBrown2.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::adi::DigitalOut Clamp(CLAMP);
	pros::adi::DigitalOut GreenFN(GREENFN);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	LadyBrown2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// get a path used for pure pursuit
// this needs to be put outside a function
//ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
#define INTAKE_PORT 21
#define LADYBROWN_PORT -6
#define LADYBROWN2_PORT 5
#define CLAMP 'B'
#define GREENFN 'A'


pros::adi::DigitalOut Clamp(CLAMP);
pros::adi::DigitalOut GreenFN(GREENFN);
pros::Motor Intake(INTAKE_PORT);
void autonomous() {
	//chassis.calibrate();
	//imu.is_calibrating();
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
	//chassis.calibrate();
	//imu.is_calibrating();
	//chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	
	chassis.setPose(88, 12, 209);
 	chassis.moveToPoint(96, 24, 5000, {.forwards = false});
 	pros::delay(400);
 	Clamp.set_value(true);
 	pros::delay(250);
	Intake.move_velocity(600);
 	pros::delay(400);
	chassis.moveToPoint(118, 48, 5000, {.maxSpeed = 50});
 	pros::delay(400);
 	chassis.turnToHeading(180, 1000);
 	chassis.moveToPoint(119, 27, 5000, {.maxSpeed = 50});
 	pros::delay(200);
	chassis.turnToHeading(164, 3000);
	pros::delay(300);
 	chassis.moveToPoint(119, 20, 5000, {.maxSpeed = 50});
 	pros::delay(300);
 	chassis.turnToHeading(315, 2000);
 	pros::delay(300);
 	chassis.moveToPoint(132, 12, 5000, {.forwards = false});
 	Clamp.set_value(false);
 	pros::delay(300);
 	chassis.moveToPoint(74, 19, 5000, {.maxSpeed = 50});
 	pros::delay(500);
 	//chassis.turnToHeading(92, 2000);
	chassis.moveToPose(47, 19, 89, 5000, {.forwards = false});
 	//chassis.moveToPoint(47, 19, 5000, {.forwards = false, .maxSpeed = 50});
 	pros::delay(500);
 	Clamp.set_value(true);
 	pros::delay(600);
 	chassis.moveToPoint(40, 40, 5000);
 	pros::delay(400);
 	chassis.moveToPoint(26, 40, 5000);
 	pros::delay(400);
 	chassis.turnToHeading(177, 3000);
 	pros::delay(400);
 	chassis.moveToPoint(26, 25, 5000);
 	pros::delay(400);
 	chassis.moveToPoint(26, 15, 5000);
 	pros::delay(400);
 	chassis.moveToPoint(10, 5, 5000, {.forwards = false, .maxSpeed = 50});
	pros::delay(400);
	Clamp.set_value(false);
	pros::delay(400);
	chassis.moveToPoint(30, 68, 5000);
	pros::delay(300);
	chassis.turnToHeading(45, 3000);
	pros::delay(300);
	chassis.moveToPoint(45, 93, 5000);
	Intake.move_velocity(0);
	chassis.turnToHeading(225, 2000);



	 //chassis.moveToPoint(0, 48, 5000);
	 //chassis.turnToHeading(90, 5000);
     //chassis.moveToPose(0, 24, 0, 4000);
}

/**
 * Runs in driver control
 */
#define INTAKE_PORT 21
#define LADYBROWN_PORT -6
#define LADYBROWN2_PORT 5
#define CLAMP 'B'
#define GREENFN 'A'
void opcontrol() {
	pros::Motor Intake(INTAKE_PORT);
	Intake.set_gearing(pros::E_MOTOR_GEAR_BLUE);
	Intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor LadyBrown(LADYBROWN_PORT);
	LadyBrown.set_gearing(pros::E_MOTOR_GEAR_GREEN);
	LadyBrown.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor LadyBrown2(LADYBROWN2_PORT);
	LadyBrown2.set_gearing(pros::E_MOTOR_GEAR_GREEN);
	LadyBrown2.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::adi::DigitalOut Clamp(CLAMP);
	pros::adi::DigitalOut GreenFN(GREENFN);
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	LadyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	LadyBrown2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
         int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        chassis.tank(leftY, rightY);
        // delay to save resources
        pros::delay(10);
		
		if(master.get_digital(DIGITAL_R2)){
			Intake.move_velocity(600);
		}
		else if(master.get_digital(DIGITAL_L2)){
			Intake.move_velocity(-600);
		}
		else{
			Intake.move_velocity(0);
		}
		static bool toggle { false };
if (master.get_digital_new_press(DIGITAL_B)) {
    if (!toggle) {
        Clamp.set_value(true);
        toggle = !toggle;
    }
    else {
        Clamp.set_value(false);
        toggle = !toggle;
    }
}
static bool toggle2 { false };
if (master.get_digital_new_press(DIGITAL_DOWN)) {
    if (!toggle2) {
        GreenFN.set_value(true);
        toggle2 = !toggle2;
    }
    else {
        GreenFN.set_value(false);
        toggle2 = !toggle2;
    }
}
if(master.get_digital(DIGITAL_R1)){
	LadyBrown.move_velocity(200);
	LadyBrown2.move_velocity(200);
}
else if(master.get_digital(DIGITAL_L1)){
	LadyBrown.move_velocity(-200);
	LadyBrown2.move_velocity(-200);
}
else{
	LadyBrown.move_velocity(0);
	LadyBrown2.move_velocity(0);
}


if(master.get_digital(DIGITAL_Y)){
	LadyBrown.move_absolute(205, 200);
	LadyBrown2.move_absolute(205, 200);
}
else if(master.get_digital(DIGITAL_RIGHT)){
	LadyBrown.move_absolute(0, 200);
	LadyBrown2.move_absolute(0, 200);
}

    }
}