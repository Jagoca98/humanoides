// teo_plantilla_cpp controller.

#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/Motor.hpp>

#include <iostream>
#include <math.h>

#define MY_PI 3.14159265358979323846
#define MAX 1000

void mySleep(int sleepMs); // Implementation after main()
double radians(const double degrees) { return degrees*M_PI/180.0; }
double degrees(const double radians) { return radians*180.0/M_PI; }

const double l0 = 0.329;
const double l1 = 0.215+0.090;
const double A = l0+l1;
const double r = 0.07;


void fwdKin(const double q0, const double q1, double& x, double& z)
{
  // __1__
  // std::cout << "fwdKin: input (q0 q1): " << q0 << " " << q1 << std::endl;
  double u = l0*cos(q0)+l1*cos(q0+q1);
  double v = l0*sin(q0)+l1*sin(q0+q1);
  // std::cout << "fwdKin: intermediate (u v): " << u << " " << v << std::endl;
  x = v;
  z = A-u;
  // std::cout << "fwdKin: output (x z): " << x << " " << z << std::endl;
  return;
}

void invKin(const double x, const double z, double& q0, double& q1)
{
  // __2__
  // std::cout << "invKin: input (x z):" << x << " " << z << std::endl;
  double u = A-z;
  double v = x;
  // std::cout << "invKin: intermediate (u v): " << u << " " << v << std::endl;
  q1 = acos((pow(u,2)+pow(v,2)-pow(l0,2)-pow(l1,2))/(2*l0*l1));
  q0 = atan2(v,u)-atan2(l1*sin(q1),(l0+l1*cos(q1)));
  // std::cout << "invKin: output (q0 q1): " << degrees(q0) << " " << degrees(q1) << std::endl;
  return;
}

int main(int argc, char **argv)
{
  // create the Robot instance.
  webots::Robot *robot = new webots::Robot();

  webots::Motor* motor_q0 = robot->getMotor("r_shoulder_pitch");
  webots::Motor* motor_q1 = robot->getMotor("r_elbow_pitch");
  // webots::Motor* motor_q2 = robot->getMotor("r_shoulder_roll");

  double theta [MAX];
  double target_x[MAX];
  double target_z[MAX];
  double target_q0[MAX];
  double target_q1[MAX];

  // target 0

  //// target 0: set initial Position

  // __3__
  double target0_q0 = radians(30); // use helper: radians()
  double target0_q1 = radians(50); // use helper: radians()

  //// target 0: perform fwdKin

  double target0_x, target0_z;
  fwdKin(target0_q0, target0_q1, target0_x, target0_z);
  
  
  for(int i=0; i<MAX; i++){
    theta[i]=2*MY_PI*i/(MAX-1);
    target_x[i] = target0_x + r*cos(theta[i]);
    target_z[i] = target0_z+0.1 + r*sin(theta[i]);
    invKin(target_x[i], target_z[i], target_q0[i], target_q1[i]);
  }
 

  
  int i = 0;
   
  while(true){
    if(i>=MAX){
      i = 0;
    }
    
    motor_q0->setPosition(target_q0[i]);
    motor_q1->setPosition(target_q1[i]);
    
    if(-1 == robot->step(1000))
        break;
    // mySleep(1000);
    i++;
  }
  
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
