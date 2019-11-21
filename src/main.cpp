/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Kevin Lou                                                 */
/*    Created:      Fri Nov 9 2019                                            */
/*    Description:  3946X Code OFFICAL                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/*                               Program Notes                                */
/*    Claw motor encoder needs to be set to correct values                    */
/*    Motor speeds may need to be changed                                     */
/*    Rollers are currently spinning constantly, this may need to be changed  */
/*    Autonomous code needs to be added                                       */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

/*----------------------------------------------------------------------------*/
/*           Global Instances of motors, device, and competition             */
/*----------------------------------------------------------------------------*/

// A global instance of competition
competition Competition;

//global instances of motors and other devices here
vex::motor leftFrontDrive(vex::PORT12, vex::gearSetting::ratio18_1, false);
vex::motor rightFrontDrive(vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor leftBackDrive(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor rightBackDrive(vex::PORT7, vex::gearSetting::ratio18_1, true);
vex::motor rightLiftMotor(vex::PORT1, vex::gearSetting::ratio36_1, false);
vex::motor leftLiftMotor(vex::PORT20, vex::gearSetting::ratio36_1, true);
vex::motor clawMotor(vex::PORT21, vex::gearSetting::ratio18_1, false);
vex::motor rollerMotor(vex::PORT10, vex::gearSetting::ratio18_1, false);
vex::controller controller1 = vex::controller();

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                            User Control Task                              */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  double RightDriveVel;
  double LeftDriveVel;
  // main execution loop for the user control program.
  while (1) {
    //drive
    RightDriveVel = controller1.Axis2.value();
    LeftDriveVel = controller1.Axis3.value();
    //right drive
    rightFrontDrive.spin(directionType::fwd, RightDriveVel, velocityUnits::pct);
    rightBackDrive.spin(directionType::fwd, RightDriveVel, velocityUnits::pct);
    //left drive
    leftFrontDrive.spin(directionType::fwd, LeftDriveVel, velocityUnits::pct);
    leftBackDrive.spin(directionType::fwd, LeftDriveVel, velocityUnits::pct);

    //lift
    if(controller1.ButtonR1.pressing()){
      //lift up
      rightLiftMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      leftLiftMotor.spin(directionType::fwd, 100, velocityUnits::pct);
    }else if(controller1.ButtonR2.pressing()){
      //lift down
      if(leftLiftMotor.position(rotationUnits::deg)>-30){
        rightLiftMotor.spin(directionType::rev, 35, velocityUnits::pct);
        leftLiftMotor.spin(directionType::rev, 35, velocityUnits::pct);
      }else{
        rightLiftMotor.spin(directionType::rev, 100, velocityUnits::pct);
        leftLiftMotor.spin(directionType::rev, 100, velocityUnits::pct);
      }
      
    }else{
      //lift stop
      rightLiftMotor.stop(hold);
      leftLiftMotor.stop(hold);
    }

    //claw
    if(controller1.ButtonL1.pressing()){
      //claw open
      clawMotor.rotateTo(-55,rotationUnits::deg, 50, velocityUnits::pct, false);
    }else if(controller1.ButtonL2.pressing()){
      //claw close
      clawMotor.rotateTo(10,rotationUnits::deg, 50, velocityUnits::pct, false);
    }

    //rollers
      //rollerMotor.spin(directionType::fwd, 75, velocityUnits::pct);

    wait(20, msec); // Sleep the task to save resources
  }
}

/*---------------------------------------------------------------------------*/
/*                                  Main                                     */
/*---------------------------------------------------------------------------*/

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
