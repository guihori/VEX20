#include "api.h"
#include "PID.hpp"
#include <vector>
#include <math.h>

//Motors follow counter clockwise order, starting from frontR

class Drivetrain
{
private:
  //Setup private variables
  std::vector<pros::Motor> motors;
  std::vector<PID> pid_controls;
  double kp;
  double ki;
  double kd;
  double e_t;
  double tpr;
  double tpi;
  double wheel_diameter;
  double tpt;
  //Motors follow counter clockwise order, starting from frontR

public:
  Drivetrain(std::vector<int> m_ports, pros::motor_gearset_e gearset)
  {
    //Coefficients for PID controllers
    kp = .01;
    ki = 0;
    kd = -.05;
    e_t = 100;

    //Encoder ticks reported per motor revolution
    tpr = 900;
    wheel_diameter = 4;

    //Encoder ticks reported per inch traveled
    //M_PI = pi = 3.14
    tpi = tpr/(wheel_diameter*M_PI);

    //Constructing the motors and adding them to the array
    pros::Motor frontR (m_ports[0], gearset,1);
    motors.push_back(frontR);
    pros::Motor frontL (m_ports[1], gearset);
    motors.push_back(frontL);
    pros::Motor rearL (m_ports[2], gearset);
    motors.push_back(rearL);
    pros::Motor rearR (m_ports[3], gearset,1);
    motors.push_back(rearR);

    //Creating PID controllers for each motor
    for(int i = 0; i < motors.size(); i++)
    {
      PID tmp(kp, ki, kd, e_t);
      pid_controls.push_back( tmp );
    }
  }

  //Drives each motor at the same given power
  void drive(double p)
  {
    for(int i = 0; i < motors.size(); i++) {
      motors[i].move(p);
    }

  }


  //Drives motors at the same given power, with the direction reversed on each side of the robot
  //Allows the drivetrain to turn the bot at the given power
  void turn(double p)
  {
    motors[0].move(-p);
    motors[1].move(p);
    motors[2].move(p);
    motors[3].move(-p);
  }

  void mechanum(double fr, double fl, double bl, double br)
  {
    motors[0].move(fr);
    motors[1].move(fl);
    motors[2].move(bl);
    motors[3].move(br);
  }


  //Uses a PID loop to drive the motor a specified number of ticks
  void drive_ticks(double ticks, std::vector<int> dirs, int max_power = 127, double timeout = 5000)
  {

    //Update the PID controllers with the new targers
    for(int i = 0; i < motors.size(); i++)
    {
      pid_controls[i].update_target(ticks*dirs[i]+motors[i].get_position());
    }

    //Begin PID loop
    double dt = 2;
    double passed_time = 2;
    while(!check_arrived())
    {

      //For each motor, find the output power according to the PID controls
      //If the movement just started, keep the power below a specified value
      //Otherwise, cap it at 1 and multiply it by the specified max power.
      for(int i = 0; i < motors.size(); i++)
      {
        double output = pid_controls[i].update(motors[i].get_position(), dt);
        double dir = abs(output)/output;
        if(abs(output) >= 1 && passed_time <= 400) {
          output = dir*passed_time/400;
        }
        printf("output: %f\npassed_time: %f\n", output, passed_time);
        if(abs(output) > 1) {
          output = dir;
        }
        motors[i].move(output*max_power);
      }

      //If the loop has taken longer than the specified timeout, break out of the loop.
      if(passed_time >= timeout) {
        break;
        printf("breaking!\n");
      }
      passed_time = passed_time+dt;
      pros::delay(dt);
    }
    printf("done moving!\n");
    drive(0);

  }

  //Convert inches to ticks using the calculated ticks/inch value, and drive that many ticks
  void drive_inches(double inches,double max_power = 127, double timeout = 5000)
  {
    std:: vector<int> dirs {1, 1, 1, 1};
    drive_ticks(inches*tpi,dirs, max_power, timeout);
  }

  //Check whether or not all the motors are at their target encoder positions.
  bool check_arrived()
  {
    bool arrived = true;
    for(int i = 0; i < motors.size(); i++) {
      arrived = arrived && pid_controls[i].check_arrived();
    }
    return arrived;
  }

  //Prints encoder positions for each motor. Used for debugging.
  void print_position() {
    for(int i = 0; i < motors.size(); i++) {
      printf("Motor %d pos: %f\n", i, motors[i].get_position());
    }

  }

  //Uses a PID loop to turn a specified number of degrees
  //Uses an PID controller attached to the rotational reading of an IMU to control turning and turns the motors based on that readout
  void turn_degrees(double degrees, pros::Imu *imu, double timeout = 5000, double max_power = 50)
  {
    //PID coefficients for turning
    double kp = .35;
    double ki = .000001;
    double kd = -.005;
    double e_t = 1;
    degrees = -degrees;

    //Update the PID controller with the new target
    double initial_rot= imu->get_rotation();
    PID turn_ctrl(kp, ki, kd, e_t);
    turn_ctrl.update_target(degrees+initial_rot);

    //Creates a vector for the directions that each motor should turn.
    std::vector<int> dirs = {-1, 1, 1, -1};

    //Begin PID loop
    double dt = 2;
    double passed_time = 0;
    double goal_time = 0;
    while(goal_time < 100)
    {
      //Find the output for each motor according to the current IMU rotation value
      double output = turn_ctrl.update(imu->get_rotation(), dt);
      double dir = abs(output)/output;
      if(abs(output) > 1) {
        output = dir;
      }

      //Turn the motors according to that value
      for(int i = 0; i < motors.size(); i++) {
        motors[i].move(output*max_power*dirs[i]);
      }

      //If the loop takes longer than a specified timeout, break out of it.
      if(passed_time >= timeout) {
        break;
        printf("breaking!\n");
      }
      pros::delay(dt);
      passed_time = passed_time+dt;

      //Counts how long the robot is near its target position for
      if(abs(imu->get_rotation() - initial_rot - degrees) < e_t) {
        goal_time += dt;
      }
      else {
        goal_time = 0;
      }
    }
    printf("done moving!\n");
    drive(0);
  }
};
