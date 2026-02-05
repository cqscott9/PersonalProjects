/******************************************************************************************
  AUTHOR: Bruh

  PURPOSE: Sinusoidal position trajectories:
  Motor 0 oscillates between -3.14 and 3.14 rad following sin(t)
  Motor 1 follows the same trajectory with a phase offset of π/2 (quarter period).

  UPDATED: 12/04/2025
******************************************************************************************/

#include "sinusoidal_trajectory.h"
#include <math.h>

// Trajectory parameters
static const float AMPLITUDE0 = 3.14f;
static const float AMPLITUDE1 = 3.14f;
static const float FREQUENCY0 = 1.0f;
static const float FREQUENCY1 = 1.0f;
static const float PHASESHIFT = PI;
static const float OMEGA0 = 2.0f * PI * FREQUENCY0; // Angular frequency [rad/s]
static const float OMEGA1 = 2.0f * PI * FREQUENCY1;

// Control gains
static const float KP = 2.0f; // Position gain
static const float KD = 0.5f; // Damping gain

// Time tracking
static float elapsed_time = 0.0f; 


/******************************************************************************************
  FUNCTION: Update the sinusoidal trajectory for both motors, Call in the deterministic
  loop with the time step
  
  ARGS:
    motors: Array containing the two Motor objects
    dt: Time step in seconds
******************************************************************************************/
void update_sinusoidal_trajectory(Motor* motors, float dt) {
  
  elapsed_time += dt;
  
  // Sine equation to calculate desired trajectory
  float motor0_position = AMPLITUDE0 * sin(OMEGA0 * elapsed_time);
  float motor1_position = AMPLITUDE1 * sin(OMEGA1 * elapsed_time + PHASESHIFT);
  
  // Send position commands to motors
  motors[0].set_position(motor0_position, KP, KD);
  motors[1].set_position(motor1_position, KP, KD);
}
