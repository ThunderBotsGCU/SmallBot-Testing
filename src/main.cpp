#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

vex::competition Competition;

// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
controller Controller1 = controller(primary);
motor LeftDriveMotor = motor(PORT11, ratio18_1, false);

motor RightDriveMotor = motor(PORT12, ratio18_1, false);

inertial Inertial20 = inertial(PORT20);



// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

// Constants for autonomous driving
const double WHEEL_DIAM_IN = 4.0; // 4 inch wheels (adjust based on your robot)

// PD controller constants for heading correction
const double KP_HEADING = 0.8;    // Proportional gain - tune this value
const double KD_HEADING = 0.0;    // Derivative gain - tune this value

// PD controller function for heading correction
double calculateHeadingCorrection(double target_heading, double &prev_error) {
    double current_heading = Inertial20.heading(degrees);
    double error = target_heading - current_heading;
    
    // Handle angle wrap-around (e.g., from 359° to 1°)
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    // Calculate derivative term
    double derivative = error - prev_error;
    
    // PD calculation
    double correction = (KP_HEADING * error) + (KD_HEADING * derivative);
    
    // Update previous error for next iteration
    prev_error = error;
    
    return - correction;
}

void driveStraight(double distance, double acceleration, double end_speed = 0) {
    // Record starting encoder positions
    double left_start = LeftDriveMotor.position(degrees);
    double right_start = RightDriveMotor.position(degrees);
    
    // Record starting heading for PD control
    double target_heading = Inertial20.heading(degrees);
    double prev_heading_error = 0;

    // Motion profile setup
    double max_speed = 80; // percent power cap, tune this
    double start_speed = 0;  // assume we start from rest
    double target_dist = fabs(distance); // inches

    double pos = 0;
    double speed = start_speed;

    while (pos < target_dist) {
        // Compute how much distance remains
        double dist_remaining = target_dist - pos;
        Brain.Screen.clearScreen();
        Brain.Screen.print("%.2f", dist_remaining);
        // Decide if we should accelerate or decelerate
        double stopping_dist = (speed * speed - end_speed * end_speed) / (2 * acceleration);

        if (dist_remaining <= stopping_dist) {
            // Decelerate
            speed -= acceleration * 0.02; // dt = 20 ms
            if (speed < end_speed) speed = end_speed;
        } else {
            // Accelerate
            speed += acceleration * 0.02;
            if (speed > max_speed) speed = max_speed;
        }

        // Calculate heading correction using PD controller
        double heading_correction = calculateHeadingCorrection(target_heading, prev_heading_error);
        
        // Apply speed to motors with heading correction
        double direction = (distance >= 0) ? 1 : -1;
        double left_speed = direction * speed - heading_correction;
        double right_speed = direction * speed + heading_correction;
        
        LeftDriveMotor.spin(reverse, left_speed, pct);
        RightDriveMotor.spin(forward, right_speed, pct);

        // Update pos estimate from encoders (relative to start position)
        double left_deg = fabs(LeftDriveMotor.position(degrees) - left_start);
        double right_deg = fabs(RightDriveMotor.position(degrees) - right_start);
        double avg_deg = (left_deg + right_deg) / 2.0;
        pos = (avg_deg / 360.0) * (M_PI * WHEEL_DIAM_IN);

        wait(20, msec);
    }
    LeftDriveMotor.setVelocity(0, percent);
    RightDriveMotor.setVelocity(0,percent);
    // Stop motors at the end
    LeftDriveMotor.stop(brake);
    RightDriveMotor.stop(brake);
}

// Autonomous function that drives forward exactly 50 inches
void autonomous() {
    // Calibrate inertial sensor
    Inertial20.calibrate();
    waitUntil(!Inertial20.isCalibrating());
    
    // Drive forward 50 inches with acceleration of 40 units/second^2
    driveStraight(150.0, 40.0, 0.0);
}

// Driver control function for teleop
void usercontrol() {
    while (true) {
        double RightDrivePower = Controller1.Axis2.position();
        double LeftDrivePower = Controller1.Axis3.position();

        LeftDriveMotor.setVelocity(LeftDrivePower, percent);
        RightDriveMotor.setVelocity(RightDrivePower, percent);
        LeftDriveMotor.spin(reverse);
        RightDriveMotor.spin(forward);
        wait(20, msec);
    }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Set up callbacks for autonomous and driver control periods
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Prevent main from exiting with an infinite loop
  while (true) {
    wait(100, msec);
  }
}
