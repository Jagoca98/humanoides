// teo_plantilla_cpp controller.

#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include <iostream>
#include <math.h>
#include <limits>

#define MY_PI 3.14159265358979323846
#define MAX 1000
#define Vmax 0.5
#define W 1000

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

void angle_corrector(const double &angle, double &angle_corrected){
  angle_corrected = angle - int(angle/(2*MY_PI))*2*MY_PI;
}

int main(int argc, char **argv)
{
  // create the Robot instance.
  webots::Robot *robot = new webots::Robot();

  webots::Motor* motor_q0 = robot->getMotor("r_shoulder_pitch");
  webots::Motor* motor_q1 = robot->getMotor("r_elbow_pitch");
  // webots::Motor* motor_q2 = robot->getMotor("r_shoulder_roll");
  
  int timeStep = (int)robot->getBasicTimeStep();
  
  const int number = robot->getNumberOfDevices();
  std::cout << number << std::endl;
  
  for(int i=0; i<number;i++){
    auto* tag = robot->getDeviceByIndex(i);
    std::string nombre = tag->getName();
    std::cout << "Index " << i << " is a " << nombre << std::endl;
  }
  
  webots::PositionSensor* encoder_elbow = robot->getPositionSensor("r_elbow_pitch_sensor");
  webots::PositionSensor* encoder_shoulder = robot->getPositionSensor("r_shoulder_pitch_sensor");
  encoder_shoulder->enable(timeStep);
  encoder_elbow->enable(timeStep);

  double theta [MAX];
  double target_q0;
  double target_q1;
  double velocity[MAX];

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
    velocity[i] = Vmax*sin(W*theta[i]);
  }


  motor_q0->setVelocity(0.0);
  motor_q1->setVelocity(0.0);
  motor_q0->setPosition(std::numeric_limits<double>::infinity());
  motor_q1->setPosition(std::numeric_limits<double>::infinity());

  int i = 0;
   
  while(true){
    if(i>=MAX){
      i = 0;
    }
    
    double theta0 = encoder_shoulder->getValue();
    double theta1 = encoder_elbow->getValue();
    angle_corrector(theta0, target_q0);
    angle_corrector(theta1, target_q1);
    std::cout << "targe q0: " <<target_q0 << ", target q1: " << target_q1 << std::endl;
    
    double J_inv11 = -sin(target_q0 + target_q1)/(l0*cos(target_q0 + target_q1)*sin(target_q0) - l0*sin(target_q0 + target_q1)*cos(target_q0));
    double J_inv12 = cos(target_q0 + target_q1)/(l0*cos(target_q0 + target_q1)*sin(target_q0) - l0*sin(target_q0 + target_q1)*cos(target_q0));
    double J_inv21 = (l1*sin(target_q0 + target_q1) + l0*sin(target_q0))/(l0*l1*cos(target_q0 + target_q1)*sin(target_q0) - l0*l1*sin(target_q0 + target_q1)*cos(target_q0));
    double J_inv22 = -(l1*cos(target_q0 + target_q1) + l0*cos(target_q0))/(l0*l1*cos(target_q0 + target_q1)*sin(target_q0) - l0*l1*sin(target_q0 + target_q1)*cos(target_q0));
    double w0 =  J_inv11 * velocity[i] + J_inv12 * velocity[i];
    double w1 =  J_inv21 * velocity[i] + J_inv22 * velocity[i];
    // std::cout << w0 << ", " << w1 << std::endl;
    
    
    motor_q0->setVelocity(w0);
    motor_q1->setVelocity(w1);
    
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
