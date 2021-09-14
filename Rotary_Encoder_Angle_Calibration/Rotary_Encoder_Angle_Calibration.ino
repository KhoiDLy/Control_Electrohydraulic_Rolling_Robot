#include <ADC_Module.h>
#include <ADC.h>

ADC *adc = new ADC();

byte segmentClock =  4; 
byte segmentLatch =  2; 
byte segmentData =  3; 
byte segmentGbar = 5;
byte segmentClr = 6;

int pin_Sync = 32;

byte pin_5Ven = 38;
byte pin_24Ven = 37;
byte pin_HVen = 36;

byte adc_HVmon = A1;
byte adc_Imon = A0;

byte dac_Vprog = A22;

int number = 0; // LED number

const int PIN_CS = 9;
const int PIN_CLOCK = 7;
const int PIN_DATA = 8;

float angle;
void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  
  pinMode(PIN_DATA, INPUT);

  digitalWrite(PIN_CLOCK, HIGH);
  digitalWrite(PIN_CS, LOW);

  delay(1000);

  pinMode(pin_Sync,OUTPUT);
  
  pinMode(segmentClock, OUTPUT);
  pinMode(segmentData, OUTPUT);
  pinMode(segmentLatch, OUTPUT);
  pinMode(segmentGbar, OUTPUT);
  pinMode(segmentClr, OUTPUT);

  // Initializing HV_enable pin
  pinMode(pin_HVen, OUTPUT);
  digitalWrite(pin_HVen, LOW);

  // Turning off 24V output
  pinMode(pin_24Ven, OUTPUT);
  digitalWrite(pin_24Ven, LOW);
  Serial.println("24V Voltage Converters is Off ...");
  delay(500);

  // starting 5V output
  pinMode(pin_5Ven, OUTPUT);
  digitalWrite(pin_5Ven, HIGH);
  Serial.println("5V Voltage Converters Initialized ...");
  delay(500);
  
  // Set the DAC voltage to zero
  analogWriteResolution(12);
  analogWrite(dac_Vprog, 0);
  Serial.println("Programmable Voltage is 0 V ...");
  delay(500);

}


//byte stream[16];
void loop() {
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(2);
  digitalWrite(PIN_CS, LOW);
  int pos = 0;
  for (int i=0; i<10; i++) {
    digitalWrite(PIN_CLOCK, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_CLOCK, HIGH);
   
    byte b = digitalRead(PIN_DATA) == HIGH ? 1 : 0;
    pos += b * pow(2, 10-(i+1));
  }
  for (int i=0; i<6; i++) {
    digitalWrite(PIN_CLOCK, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_CLOCK, HIGH);
  }
  digitalWrite(PIN_CLOCK, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_CLOCK, HIGH);
  angle = 360-pos*360.00/510.00-193.41;
  if (angle  >= -194.12 && angle < 0) {
    angle = map(angle,-194.12,0,167.29,360);
  }
  Serial.println(angle);
}
