#include <string.h>
#include <ADC_Module.h>
#include <ADC.h>
ADC *adc = new ADC();

// Sensing parameters for the wheel
int q_state = 0;
int pre_q_state = 0;
float angle_Old = 0;
float angle = 0.00;

byte segmentClock =  4;
byte segmentLatch =  2;
byte segmentData =  3;
byte segmentGbar = 5;
byte segmentClr = 6;

byte pin_5Ven = 38;
byte pin_24Ven = 37;
byte pin_HVen = 36;

byte adc_HVmon = A1;
byte adc_Imon = A0;

byte dac_Vprog = A22;

const int PIN_CS = 9;
const int PIN_CLOCK = 7;
const int PIN_DATA = 8;

// Defining sampling rate for the rotary encoder data collection
#define SAMPLING_FREQUENCY 300  //Hz
unsigned long sampling_period_us;
unsigned long time_stamp;
unsigned long counter = 0; // This counts the number of loop iteration before the wheel begins rolling (suggesting to make 5 seconds out of this
unsigned long dummy_data = 1111111;
// Defining the actuator parameters
int actuator_idx = 1; // This has a value from 0 to 15
int actuation_counter = 1;  // Simply counts how many activation has been produced. Has a range of 1 to infinity
int start_phase = 6; // the starting_phase contains the first x number of actuators to be activated at full potentials (3 degree activation).
int cont_phase_act_angle = 6; // continuous_phase_activation_angle is the activation angle of all actuators in the continuous phase (after the starting phase)
int cont_phase_act_dur = 15; // cont_phase_act_angle + cont_phase_act_dur = 20

float *input_cmd; // The variable to receive data from the computer to update the control command
bool recv_cmd_FLAG = false;
bool update_cmd_FLAG = false;
bool onetozero_done_FLAG = false;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // Initilizing pins for the rotary encoder
  pinMode(PIN_CS, OUTPUT); // Rotary_encoder CS pin
  pinMode(PIN_CLOCK, OUTPUT);  // Rotary_encoder CS pin
  pinMode(PIN_DATA, INPUT);  // Rotary_encoder Datapin

  digitalWrite(PIN_CLOCK, HIGH);
  digitalWrite(PIN_CS, LOW);

  // Initializing pins for the LED drivers
  pinMode(segmentClock, OUTPUT);
  pinMode(segmentData, OUTPUT);
  pinMode(segmentLatch, OUTPUT);
  pinMode(segmentGbar, OUTPUT);
  pinMode(segmentClr, OUTPUT);

  // starting 5V output
  pinMode(pin_5Ven, OUTPUT);
  digitalWrite(pin_5Ven, HIGH);
  //  Serial.println("5V Voltage Converters Initialized ...");

  // Set the DAC voltage to zero
  analogWriteResolution(12);
  analogWrite(dac_Vprog, 0);

  // Turning on 24V output
  pinMode(pin_24Ven, OUTPUT);
  digitalWrite(pin_24Ven, HIGH);
  //  Serial.println("24V Voltage Converters is on ...");

  // Initializing HV_enable pin
  pinMode(pin_HVen, OUTPUT);
  digitalWrite(pin_HVen, HIGH); // Placing this after the dac_Vprog causes system to mess up!
  //  Serial.println("HV enable pin is off ...");

  // Initializing the IR Led drivers
  digitalWrite(segmentClock, LOW);
  digitalWrite(segmentData, LOW);
  digitalWrite(segmentLatch, LOW);
  digitalWrite(segmentGbar, HIGH);
  digitalWrite(segmentClr, LOW); // Clear the memory
  delay(1000);
  digitalWrite(segmentClr, HIGH);

  // Set the DAC voltage to 9kV
  analogWrite(dac_Vprog, 2800);
  //  Serial.println("Programmable Voltage is 0 V ...");
  delay(1000);

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  //Total delay time is about 2 seconds


  // Operation starts here
  angle_Old = angle;
  angle = absoluteAngle();
  digitalWrite(segmentGbar, LOW);
}

void loop() {
  time_stamp = micros();
  SendingData(time_stamp, actuator_idx + 1, angle, q_state);
  pre_q_state = q_state;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////// Updating the new input command //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (Serial.available() > 0) {
    // How to access element:
    //*(input_cmd + 0): target idx;
    //*(input_cmd + 1): activation angle;
    //*(input_cmd + 2): activation duration;
    input_cmd = StringtoInput_cmd();
    recv_cmd_FLAG = true;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (recv_cmd_FLAG == true && update_cmd_FLAG == true) {
    actuator_idx = input_cmd[0]; // Assigning the target index from input_cmd to actuator_idx
    cont_phase_act_angle = input_cmd[1]; // Assigning the activation_angle from input_cmd to the act_angle
    cont_phase_act_dur = input_cmd[2] - input_cmd[1]; // Assigning the deactivation_angle from input_cmd to the activation duration
    recv_cmd_FLAG = false;
    update_cmd_FLAG = false;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////// input implementation to the physical robot//////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (counter > 3000) { // Do nothing for 8 seconds
    if (actuation_counter <= start_phase) {
      if ( (angle >= actuator_idx * 22.5 + 3) && angle <= (actuator_idx * 22.5 + 20)) { // If the angle is within 3 to 20 degree and the number of activation is still within start_phase value
        latchFunction(actuator_idx + 1); q_state = 1;
      }
      else if ( angle > (actuator_idx * 22.5 + 20) || angle < 3) { // As soon as the angle exceeds the activation range
        latchFunction(0); q_state = 0;
        if (actuation_counter < start_phase) {
          actuator_idx = (actuator_idx  + 1) % 16; // Increment the actuation index
        }
        else if (actuation_counter == start_phase && onetozero_done_FLAG == false) {
          update_cmd_FLAG = true;
        }
        actuation_counter = actuation_counter + 1; // Increment the activation counter to compare with the number of activations in start_phase
      }
      else {
        // Do nothing... (after actuator_idx increments, angle is much less than the idx*22.5+angles)
      }
    }
    else {
      if ( (angle >= actuator_idx * 22.5 + cont_phase_act_angle) && angle <= (actuator_idx * 22.5 + cont_phase_act_angle + cont_phase_act_dur)) { // After the end of start phase
        latchFunction(actuator_idx + 1); q_state = 1;
      }
      else if ( angle > (actuator_idx * 22.5 + cont_phase_act_angle + cont_phase_act_dur) && onetozero_done_FLAG == false) { // As soon as the angle exceeds the activation range
        update_cmd_FLAG = true;
        latchFunction(0); q_state = 0;
        //actuator_idx = (actuator_idx  + 1) % 16; // Increment the actuation index
      }
      else {
        // Do nothing... (after actuator_idx increments, angle is much less than the idx*22.5+angles)
      }
    }

    // Ensuring that the else if condition for angle only run once (preventing the update_cmd_FLAG to turn TRUE multiple times)
    if ((pre_q_state == 1 || pre_q_state == 0) && q_state == 0) {
      onetozero_done_FLAG = true;
    }
    else {
      onetozero_done_FLAG = false;
    }

    ////////////////////////////////////////// Obtaining the best angular position without errors //////////////////////////////////////////////////
    angle = absoluteAngle();
    while ((angle > angle_Old + 0.9 || angle < angle_Old - 0.9) && angle_Old < 359.9 && angle_Old > 0.1) { // change if to while for better reliability
      angle = absoluteAngle();
      angle_Old = angle;
    }
    angle_Old = angle;

//    if ((angle > angle_Old + 0.9 || angle < angle_Old - 0.9) && angle_Old < 359.9 && angle_Old > 0.1) {
//      angle = absoluteAngle();
//      angle_Old = angle;
//    }
//    else (angle_Old = angle);
//
//    if ((angle > angle_Old + 0.9 || angle < angle_Old - 0.9) && angle_Old < 359.9 && angle_Old > 0.1) {
//      angle = absoluteAngle();
//      angle_Old = angle;
//    }
//    else (angle_Old = angle);
//
//    if ((angle > angle_Old + 0.9 || angle < angle_Old - 0.9) && angle_Old < 359.9 && angle_Old > 0.1) {
//      angle = absoluteAngle();
//      angle_Old = angle;
//    }
//    else (angle_Old = angle);

  }
  counter = counter + 1;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  LoopDelay(time_stamp);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Below is for all functions ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float absoluteAngle() {
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(3);
  digitalWrite(PIN_CS, LOW);
  int pos = 0;
  for (int i = 0; i < 10; i++) {
    digitalWrite(PIN_CLOCK, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_CLOCK, HIGH);

    byte b = digitalRead(PIN_DATA) == HIGH ? 1 : 0;
    pos += b * pow(2, 10 - (i + 1));
  }
  for (int i = 0; i < 6; i++) {
    digitalWrite(PIN_CLOCK, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_CLOCK, HIGH);
  }
  digitalWrite(PIN_CLOCK, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_CLOCK, HIGH);
  float angle = 360 - pos * 360.00 / 510.00 - 193.41;
  if (angle  >= -194.12 && angle < 0) {
    angle = map(angle, -194.12, 0, 167.29, 360);
  }
  return angle;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Latch the input - Split the input into two parts
// U9 handles D1-D8 and U8 handles D9-D16
void latchFunction(int value)
{
  if (value <= 8)
  {
    clockFunction(value); // goes into U9
    clockFunction(0); // goes into U8
  }
  else if (value > 8)
  {
    clockFunction(0);
    clockFunction(value - 8);
  }
  digitalWrite(segmentLatch, LOW);//Latch the current segment data
  delayMicroseconds(3);
  digitalWrite(segmentLatch, HIGH); //Register moves storage register on the rising edge of RCK
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void clockFunction(byte number)
{
  byte segments;
  switch (number)
  {
    case 1: segments = 1 << 0; break; // 00000001
    case 2: segments = 1 << 1; break; // 00000010
    case 3: segments = 1 << 2; break; // 00000100
    case 4: segments = 1 << 3; break; // 00001000
    case 5: segments = 1 << 4; break; // 00010000
    case 6: segments = 1 << 5; break; // 00100000
    case 7: segments = 1 << 6; break; // 01000000
    case 8: segments = 1 << 7; break; // 10000000
    case 0: segments = 0; break;      // 00000000
  }

  //Clock these bits out to the drivers
  for (byte x = 0 ; x < 8 ; x++)
  {
    digitalWrite(segmentClock, LOW);
    delayMicroseconds(3);
    digitalWrite(segmentData, segments & (1 << (7 - x)));
    delayMicroseconds(3);
    //Data transfers to the register on the rising edge of SRCK
    digitalWrite(segmentClock, HIGH);
    delayMicroseconds(3);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float *StringtoInput_cmd() {
  String incoming_str = "";
  int input_idx = 0;
  static float my_cmd[3];

  while ( Serial.available() > 0 ) {
    char ch = Serial.read();
    if (ch != ','  && ch != '\n') {
      incoming_str += ch;
    }
    else {
      my_cmd[input_idx] = incoming_str.toFloat();
      incoming_str = ""; //Reset the incoming_str
      input_idx++; // Incrementing the input_cmd index
    }
  }
  return my_cmd;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SendingData(long Wheel_time, int Wheel_actuator_idx, float Wheel_angle, int Wheel_q_state) {
  Serial.print(Wheel_time); Serial.print(",");
  Serial.print(Wheel_actuator_idx); Serial.print(",");
  Serial.print(Wheel_angle, 2); Serial.print(",");
  Serial.print(Wheel_q_state); Serial.print("\n");
  Serial.flush();
}

void LoopDelay(unsigned long time_stamp) {
  while (micros() < (time_stamp + sampling_period_us)) {
    //wait...
  }
}
