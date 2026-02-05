/******************************************************************************************
  AUTHOR: Rajiv Joshi

  PURPOSE: Main file for setup and loop

  VERSION: 2

  UPDATED: 11/20/2025
******************************************************************************************/

/******************************************************************************************
                                Include/Declare Statements:
******************************************************************************************/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
//
#include "sinusoidal_trajectory.h"
#include "diff_swerve.h" 
//

#include "motors.h"


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // uses CAN 1 on Teensy (CTX1 and CRX1)
CAN_message_t msg;


// Deterministic Loop Variables
IntervalTimer loopTimer;              // Timer for deterministic loop
volatile bool det_loop_flag = false;  // flag for deterministic loop


// Creating motor objects
Motor Right_Knee("Right_Knee_Motor", 0x02, "AKE60-8"); // Motor 1 object created
Motor Left_Knee("Left_Knee_Motor", 0x03, "AKE60-8");   // Motor 2 object created
Motor Motors[2] = {Right_Knee, Left_Knee};            // Array for holding all motors
size_t num_Motors = sizeof(Motors) / sizeof(Motors[0]);


float torqueValue = 0.0f;

/******************************************************************************************
  FUNCTION: Depending on message ID that is recieved we want to call the correct function 
  to unpack and read that message.
******************************************************************************************/
void canMessageReceived(const CAN_message_t &msg) {
  for (size_t i = 0; i < num_Motors; ++i) {
    if (msg.id == (uint32_t)(0x2900 + Motors[i].motor_id)) {
      Motors[i].motor_receive(msg);
      break;
    }
  }
}




/******************************************************************************************
  FUNCTION: Timer ISR - sets the deterministic loop flag, this function should be as short
  and fast as possible to avoid delaying other interrupts. So no heavy processing should be
  done here or printstatements.

  Basically when the timer overflows it will call this interrupt service routine
******************************************************************************************/
void timerISR() {
  det_loop_flag = true;
}


void setup() {

  // Serial Communication Setup
  while (!Serial) delay(10); // will pause till Serial console opens
  Serial.begin(115200);     // specify baudrate
  Serial.println("--------------------------------------------------------------------");
  Serial.println("Serial Baud Rate = 115200");
  Serial.println("Serial communcaiton setup COMPLETE...");
  //delay(2000);    // small delay [2000 ms]


  // Setup motor parameters from database
  Serial.println("--------------------------------------------------------------------");
  Serial.println("Initializing motor parameters from database...");
    // Initialize motor parameters from database after all globals are constructed
  for (size_t i = 0; i < num_Motors; ++i) {
    Motors[i].initFromDatabase();
  }
  Serial.println("Motor parameters initialization COMPLETE...");
  // delay(2000);    // small delay [2000 ms]


  // Getting Data for Motor Type
  Serial.println("--------------------------------------------------------------------");
  Serial.println(String("Motor 1 type: ") + Motors[0].motor_type);
  Serial.println(String("Motor 2 type: ") + Motors[1].motor_type);
  // delay(2000);    // small delay [1000 ms]


  // Getting Data for Motor CAN ID
  Serial.println("--------------------------------------------------------------------");
  Serial.println(String("Motor 1 CAN ID: ") + Motors[0].motor_id);
  Serial.println(String("Motor 2 CAN ID: ") + Motors[1].motor_id);
  // delay(2000);    // small delay [1000 ms]


  // CAN1 Communication Setup
  Serial.println("--------------------------------------------------------------------");
  Serial.println("Setting up CAN communication...");
  can1.begin();
  can1.setBaudRate(1000000);  // 1Mbps
  delay(2000);    // small delay [2000 ms]
  // Enable FIFO Interrupts 
  can1.enableFIFO();                      // enable First in First Out
  can1.enableFIFOInterrupt();             // Enable Interrupts
  can1.onReceive(canMessageReceived);     // When we interrupt on a revieced message from other devices
  can1.mailboxStatus();                   // setting up all mailboxes (look more into how mailboxes work for FlexCAN library)
  Serial.println("CAN communication setup COMPLETE...");
  //delay(2000);    // small delay [2000 ms]


  // Clear current Motor data/inforamtion
  comm_can_set_origin(Motors[0].motor_id, 0);
  comm_can_set_origin(Motors[1].motor_id, 0);
  for (size_t j = 0; j < (sizeof(Motors)/sizeof(Motors[0])); j++) { // looping through all motors to check if cleared
    Serial.println("--------------------------------------------------------------------");
    Serial.println( String("Motor ") + j + ": pos = " + Motors[j].pos + " [rad], vel = " + Motors[j].ERPM + " [rad/s], torque = " + Motors[j].current + " [Nm]");
    Serial.println(String("Clearing Motor ") + j + " data COMPLETE...");
    // delay(2000);    // small delay [2000 ms]
  }


  // Starting message countdown
  Serial.println("--------------------------------------------------------------------");
  Serial.println("Starting in 3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);


  // Setting up and Starting Deterministic Loop Timer
  // if (!loopTimer.begin(timerISR, 5000)) {       // 5000 microseconds = 200 Hz
  // if (!loopTimer.begin(timerISR, 10000)) {      // 10000 microseconds = 100 Hz
  if(!loopTimer.begin(timerISR, 1000)) {      // 1000 microseconds = 1000 Hz
    Serial.println("Failed to start timer!");
    while (1); // halt if timer fails to start
  }
  


} // end of setup





void loop() {

  // Check if the deterministic loop flag is set
  if (det_loop_flag) {
    noInterrupts();           // Disable interrupts during deterministic loop
    det_loop_flag = false;    // Reset flag
    interrupts();             // Re-enable interrupts

    // Loop Code Start

    // ***** DIFFERENTIAL SWERVE CONTROL *****
    update_diff_swerve_sinusoidal(Motors, 0.001f);

    // ***** MOTOR SINUSOIDAL CONTROL *****
    // update_sinusoidal_trajectory(Motors, 0.001f);

    // ***** Direct Motor Commands *****
    // Motor Data
    // Motors[0].set_torque(0.2);
    // Motors[1].set_torque(-0.2);
    // Motors[0].set_speed(18, 0.3);
    // Motors[1].set_speed(9, 0.3);
    // Motors[0].set_position(6.28, 2.0, 0.5);
    // Motors[1].set_position(-6.28, 2.0, 0.5);
    // Motors[0].print_motor_data();
    // Motors[0].motors_teleplot();

  } // end of deterministic loop check
  

} // end of main loop
