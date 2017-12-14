#include "Balanduino.h"
#include "Controller.h"
#include <Arduino.h>		// Standard Arduino header
#include <Wire.h>		// Official Arduino Wire library
#include <Kalman.h>
#include <avrpins.h>

static Kalman kalman;

// Define the parameters for the loop1 PID-controller (continuous)

double Kp_loop1 = 28.53;
double Ki_loop1 = 241.53;
double Kd_loop1 = 2.48;
double Tf_loop1 = 0.04;

double P_loop1 = 0.0;
double I_loop1 = 0.0;
double D_loop1 = 0.0;

double e_loop1_old = 0.0;

// Define the parameters for the loop2 PI-controller (continuous)
double Kp_loop2 = -0.06437;
double Ki_loop2 = -0.0296;
double Kd_loop2 = 0.0;
double Tf_loop2 = 0.0;

double P_loop2 = 0.0;
double I_loop2 = 0.0;
double D_loop2 = 0.0;
// Define the parameters for the loop2 PD-controller (new)
double Kp_loop2_PD = -0.0069;
double Kd_loop2_PD = -0.0333;
double Tf_loop2_PD = 1.0600;


double e_loop2_old = 0.0;

double balanduino_pos = 0.0;
double reference; // Reference value (In Swedish: Börvärde)

void setup()
{
  // Initialize all sensor hardwares, filters and observers  
  initialize();
  // Setup pulse generator for the setpoint 
  // First argument is the low value of the pulse
  // Second argument is the hight value of the pulse
  // Third argument is the pulse width in microseconds
  setup_setpoint_generator_pulse(0.0, 1.0, 10000000); 
}

void loop()
{
  //
  // Part 1: Sanity check - make sure that the motors are ok
  //
  checkMotors();

  //
  // Part 2: Time management - read actual time and 
  //    calculate the time since last sample time.
  //  
  unsigned long timer_us = micros(); // Time of current sample in microseconds  
  h = (double)(timer_us - pidTimer_us) / 1000000.0; // Time since last sample in seconds
  pidTimer_us = timer_us;  // Save the time of this sampleTime of last sample

  
  // Part 3: Read the angular orientation (rad) of the robot
  double theta = getTheta();

  // Drive motors if the robot is not lying down. If it is lying down, it has to be put in a vertical position before it starts to balance again.
  // When balancing it has to deviate more ±45 degrees form 0 degrees before it stops trying to balance
  if ((layingDown && (pitch < cfg.targetAngle - 10 || pitch > cfg.targetAngle + 10)) || (!layingDown && (pitch < cfg.targetAngle - 45 || pitch > cfg.targetAngle + 45))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
    // The robot is not laying down
  } else {
    // It's no longer laying down
    layingDown = false;

    //
    // Part 4: Read the longitudinal velocity (m/s) of the robot
    //
    double v = getSpeed(h);
    balanduino_pos = balanduino_pos + v * h;
    
    // loop2 PI-controller:
    // 
    // Part 5: Generate setpoint value (In Swedish: Börvärde)
    //
    reference = setpoint_generator_pulse();
    //double v_r = 0;

    // 
    // Part 6: Generate the control error for the loop2 loop, i.e. difference
    //    between the reference_value and the actual_output.
    //
    //double e_loop2 = v_r - v; // Control error
    double e_loop2 = reference - balanduino_pos; // Control error
  
    //
    // Part 7: Calculate control output (In Swedish: Styrsignal)
    //
    // Implement your PI-controller here:
    //
    // Pseudo-kod for a PID-controller
    // P = c0 * e
    // D = c1 * D + c2 * (e - eold)
    // u = P + I + D // Bestäm totala styrsignalen
    // daout("u", u) // Skriv styrsignalen till DA-omv.
    // I = I + c3 * e // Uppdatera integraldelen
    // eold = e
    
   // P_loop2 = Kp_loop2 * e_loop2;
   // I_loop2 = I_loop2 + (Ki_loop2 * h) * e_loop2;
   // double u_loop2 = P_loop2 + I_loop2;
    //new
    P_loop2 = Kp_loop2_PD * e_loop2;
    D_loop2 = (Tf_loop2_PD / (Tf_loop2_PD + h)) * D_loop2 + Kd_loop2_PD/(Tf_loop2_PD + h) * (e_loop2 - e_loop2_old);
    double u_loop2 = P_loop2 + D_loop2;
    
    
    
    //P_loop2 = Kp_loop2 * e_loop2;
    //double u_loop2 = P_loop2; // Calculate control output  
    
    // loop1 PID-controller:
    // 
    // Part 8: Generate setpoint value for loop1 controller (In Swedish: Börvärde)
    //
    double theta_r = u_loop2;

    // 
    // Part 9: Generate the control error for the loop1 loop, i.e. difference
    //    between the reference_value and the actual_output.
    //
    double e_loop1 = theta_r - theta; // Control error

    //
    // Part 10: Calculate control output (In Swedish: Styrsignal)
    //
    // Implement your PID-controller here:
    //
    // Pseudo-kod for a PID-controller
    // P = c0 * e
    // D = c1 * D + c2 * (e - eold)
    // u = P + I + D // Bestäm totala styrsignalen
    // daout("u", u) // Skriv styrsignalen till DA-omv.
    // I = I + c3 * e // Uppdatera integraldelen
    // eold = e

    P_loop1 = Kp_loop1 * e_loop1;
    D_loop1 = (Tf_loop1 / (Tf_loop1 + h)) * D_loop1 + Kd_loop1/(Tf_loop1 + h) * (e_loop1 - e_loop1_old);
    I_loop1 = I_loop1 + (Ki_loop1 * h) * e_loop1;
    double u_loop1 = P_loop1 + I_loop1 + D_loop1;
 

    
    //P_loop1 = Kp_loop1 * e_loop1;
    double u = u_loop1; // Calculate control output  
    double saturated_u = constrain(u, -12.0, 12.0); // Make sure the calculated output is -12 <= u <= 12 (Min voltage -12V, max voltage +12V)

    e_loop1_old = e_loop1;
    e_loop2_old = e_loop2;

    // 
    // Part 11: Actuate the control output by sending
    //    the control output to the motors
    //
    actuateControlSignal(saturated_u);    
  }


  // Update the motor encoder values
  //updateEncoders();
}

