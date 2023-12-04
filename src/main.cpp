#include "vex.h"
using namespace vex;

brain Brain;

// VEXcode device constructors
motor LeftMotorA = motor(PORT19, ratio18_1, true);
motor LeftMotorB = motor(PORT11, ratio18_1, true);
motor RightMotorA = motor(PORT1, ratio18_1, false);
motor RightMotorB = motor(PORT10, ratio18_1, false);
motor_group LeftMotorGroup = motor_group(LeftMotorA, LeftMotorB);
motor_group RightMotorGroup = motor_group(RightMotorA, RightMotorB);
inertial InertialSensor = inertial(PORT12);
smartdrive dt = smartdrive(LeftMotorGroup, RightMotorGroup, InertialSensor, 219.44, 320, 40, mm, 2);
controller Controller = controller(primary);
motor Roller = motor(PORT2, ratio18_1, false);
digital_out Wings = digital_out(Brain.ThreeWirePort.A);

void preAutonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  Roller.setVelocity(100, velocityUnits::pct);
  wait(1, seconds);
}

void autonomous(void) {
    Brain.Screen.clearScreen();
    Brain.Screen.print("Autonomous Code");
    dt.setDriveVelocity(50, velocityUnits::pct);
    Roller.setVelocity(100, velocityUnits::pct);
    Roller.spinFor(forward, 150, rotationUnits::deg);
    dt.driveFor(forward, 18, distanceUnits::in);
    LeftMotorGroup.spinFor(reverse, 200, rotationUnits::deg);
    dt.setDriveVelocity(100, velocityUnits::pct);
    dt.driveFor(forward, 15, distanceUnits::in);
    Roller.spinFor(reverse, 150, rotationUnits::deg);
    dt.driveFor(reverse, 11, distanceUnits::in);
    dt.turnFor(right, 180, rotationUnits::deg);
    dt.driveFor(reverse, 20, distanceUnits::in);
    dt.setDriveVelocity(50, velocityUnits::pct);
    dt.driveFor(forward, 12, distanceUnits::in);
    dt.setDriveVelocity(100, velocityUnits::pct);
    dt.driveFor(reverse, 16, distanceUnits::in);
    dt.driveFor(forward, 5, distanceUnits::in);
}

void userControl(void) {
  while (1) {
    int leftValue = Controller.Axis4.value();
    int rightValue = Controller.Axis3.value();

    // Apply deadzone to joysticks
    if (abs(leftValue) < 15) {
      leftValue = 0;
    }

    if (abs(rightValue) < 15) {
      rightValue = 0;
    }

    // Check if the button is pressed and spin the roller accordingly
    if (Controller.ButtonL1.pressing()) {
      Roller.spin(forward);
    } else if (Controller.ButtonL2.pressing()) {
      Roller.spin(reverse);
    } else {
      Roller.stop();  // Stop the roller if neither button is pressed
    }

    LeftMotorGroup.spin(forward, (rightValue + leftValue), velocityUnits::pct);
    RightMotorGroup.spin(forward, (rightValue - leftValue), velocityUnits::pct);
  }
}




int main() {
  competition Competition;

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  preAutonomous();
}