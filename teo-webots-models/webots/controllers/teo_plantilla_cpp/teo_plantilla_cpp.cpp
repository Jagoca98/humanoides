// teo_plantilla_cpp controller.

#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/Motor.hpp>

#include <iostream>
#include <math.h>

void mySleep(int sleepMs); // Implementation after main()
double radians(const double degrees) { return degrees*M_PI/180.0; }
double degrees(const double radians) { return radians*180.0/M_PI; }

const double l0 = 0.329;
const double l1 = 0.215+0.090;
const double A = l0+l1;

void fwdKin(const double q0, const double q1, double& x, double& z)
{
  // __1__
  std::cout << "fwdKin: input (q1 q2): " << q0 << " " << q1 << std::endl;
  double u = ...
  double v = ...
  std::cout << "fwdKin: intermediate (u v): " << u << " " << v << std::endl;
  x = ...
  z = ...
  std::cout << "fwdKin: output (x z): " << x << " " << z << std::endl;
  return;
}

void invKin(const double x, const double z, double& q0, double& q1)
{
  // __2__
  std::cout << "invKin: input (x z):" << x << " " << z << std::endl;
  double u = ...
  double v = ...
  std::cout << "invKin: intermediate (u v): " << u << " " << v << std::endl;
  q1 = ...
  q0 = ...
  std::cout << "invKin: output (q0 q1): " << degrees(q0) << " " << degrees(q1) << std::endl;
  return;
}

int main(int argc, char **argv)
{
  // create the Robot instance.
  webots::Robot *robot = new webots::Robot();

  webots::Motor* motor_q0 = robot->getMotor("r_shoulder_pitch");
  webots::Motor* motor_q1 = robot->getMotor("r_elbow_pitch");

  // target 0

  //// target 0: setPosition

  // __3__
  double target0_q0 = ... // use helper: radians()
  double target0_q1 = ... // use helper: radians()
  motor_q0->setPosition(target0_q0);
  motor_q1->setPosition(target0_q1);
  robot->step(1000);
  mySleep(1000);

  //// target 0: perform fwdKin

  double target0_x, target0_z;
  fwdKin(target0_q0, target0_q1, target0_x, target0_z);

  //// target 0: check if invKin works to recover original joint space targets

  double recovered0_q0, recovered0_q1;
  invKin(target0_x, target0_z, recovered0_q0, recovered0_q1);

  // target 1

  //// target 1: generate in Cartesian space, adding offset from target 0

  // __4a__
  double target1_x = ...
  double target1_z = ...

  //// target 1: compute in joint space, via invKin

  double target1_q0, target1_q1;
  invKin(target1_x, target1_z, target1_q0, target1_q1);

  // target 2

  //// target 2: generate in Cartesian space

  // __4b__
  double target2_x = ...
  double target2_z = ...

  //// target 2: compute in joint space, via invKin

  double target2_q0, target2_q1;
  invKin(target2_x, target2_z, target2_q0, target2_q1);

  // target 3

  //// target 3: generate in Cartesian space, removing offset from target 0

  // __4c__
  double target3_x = ...
  double target3_z = ...

  //// target 3: compute in joint space, via invKin

  double target3_q0, target3_q1;
  invKin(target3_x, target3_z, target3_q0, target3_q1);

  // Main loop:
  // - toggle between target 1, target 2, target 3 and back again
  // - perform simulation steps until Webots is stopping the controller
  while(true)
  {
    // __5__
    motor_q0->setPosition(target1_q0);
    motor_q1->setPosition(target1_q1);
    if(-1 == robot->step(1000))
        break;
    mySleep(1000);
    motor_q0->setPosition(...);
    motor_q1->setPosition(...);
    if(-1 == robot->step(1000))
        break;
    mySleep(1000);
    motor_q0->setPosition(...);
    motor_q1->setPosition(...);
    if(-1 == robot->step(1000))
        break;
    mySleep(1000);
    motor_q0->setPosition(target2_q0);
    motor_q1->setPosition(target2_q1);
    if(-1 == robot->step(1000))
        break;
    mySleep(1000);
  };

  // Exit cleanup code here.
  delete robot;
  return 0;
}

// Thanks `shf301` at <https://stackoverflow.com/questions/10918206/cross-platform-sleep-function-for-c>
#ifdef LINUX
  #include <unistd.h>
#endif
#ifdef WINDOWS
  #include <windows.h>
#endif

void mySleep(int sleepMs)
{
#ifdef LINUX
  usleep(sleepMs * 1000);
#endif
#ifdef WINDOWS
  Sleep(sleepMs);
#endif
}
