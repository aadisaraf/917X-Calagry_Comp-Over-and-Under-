#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"

double fwd;
double turning;
float up;
float down;
bool lifted = false;



pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup right_motors({13, 12, -11},pros::MotorGearset::blue);  
pros::MotorGroup left_motors({-3, 1, -2},pros::MotorGearset::blue); 
pros::MotorGroup intake({5,-14},pros::MotorGearset::blue);

pros::adi::DigitalOut rearLeftWings (2);
pros::adi::DigitalOut rearRightWings (1);

pros::adi::DigitalOut Raise1 (4);
pros::adi::DigitalOut Raise2 (3);


lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              5.75, // 5.75 inch track width
                              lemlib::Omniwheel::NEW_325, 
                              450, // drivetrain rpm is 360
                              5 // If you have a drift drive, we recommend starting with a value of 2, while a drivetrain with center traction wheels should start with a value of 8.
);
pros::Imu imu(10);



// lateral motion controller
lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            5, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0.1, // integral gain (kI)
                                             13, // derivative gain (kD)
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
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
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
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(path_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
  chassis.setPose(0,0,0);
  intake.move(120);
  chassis.moveToPoint(14,55,2000,{.maxSpeed = 70});
  pros::delay(2000);
  intake.move(0);
  pros::delay(500);
  chassis.moveToPoint(0, 27, 3000,{.maxSpeed = 70},false);
  chassis.moveToPoint(-18, 18, 3000,{.maxSpeed = 70},false);
  chassis.turnToHeading(-33, 1000);
  // chassis.moveToPose(-20,  22, -35, 3000,{.maxSpeed = 70},false);
  rearLeftWings.set_value(true);
  rearRightWings.set_value(true);
  chassis.moveToPoint(1 ,-2, 1000,{.forwards = false},false);
  rearLeftWings.set_value(false);
  rearRightWings.set_value(false);
  chassis.turnToHeading(-88, 1000);
  pros::delay(200);
  chassis.turnToHeading(90, 2000,{.direction = AngularDirection::CW_CLOCKWISE});
  intake.move(-100);   
  chassis.moveToPoint(24, 0,2000,{},false);
  rearLeftWings.set_value(true); 

}

/**
 * Runs in driver control
 */
void arcadeCurve(pros::controller_analog_e_t power,
                 pros::controller_analog_e_t turn, pros::Controller mast,
                 float t) {
  up = mast.get_analog(power);
  down = mast.get_analog(turn);
  fwd = (exp(-t / 10) + exp((fabs(up) - 127) / 10) * (1 - exp(-t / 10))) * up;
  turning = -1 * down;
  left_motors.move(fwd - turning);
  right_motors.move(fwd + turning);
}

void opcontrol() {
  
  	while (true) { // calls the arcade drive function
    	arcadeCurve(pros::E_CONTROLLER_ANALOG_LEFT_Y,
            pros::E_CONTROLLER_ANALOG_RIGHT_X, master, 10);

    // intake
    if (master.get_digital(DIGITAL_R2)) // intake
    {
      intake.move(127);
    }
    if (master.get_digital(DIGITAL_R1)) // outtake
    {
      intake.move(-127);
    }
    if (master.get_digital(DIGITAL_R1) == false &&
        master.get_digital(DIGITAL_R2) == false) // stop intake
    {
      intake.move(0);
    }

    if (master.get_digital(DIGITAL_L2)) // wing expand
    {
      rearRightWings.set_value(true);
    }
    else if (master.get_digital(DIGITAL_L1)) // wing expand
    {
      rearRightWings.set_value(true);
      rearLeftWings.set_value(true);
    } else // wing retract
    {
      rearRightWings.set_value(false);
      rearLeftWings.set_value(false);
    }

    // hang mechanism
    if (master.get_digital(DIGITAL_RIGHT)) {
      lifted = !lifted;
      pros::delay(300);
    }
    if (lifted) {
      Raise1.set_value(true);
      Raise2.set_value(true);
    }
    if (!lifted) {
      Raise1.set_value(false);
      Raise2.set_value(false);
    

    pros::delay(20);
  }
}
}