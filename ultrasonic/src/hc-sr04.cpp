/*
 * Code for interfacing a 32U4 with the SR-HR04 ultrasonic sensor. 
 * 
 * This uses the Input Capture feature of the ATmega32U4 (e.g., Leonardo) to get precision readings.
 * Specifically, you must connect the pulse width pin to pin 13 (ICP3) on the 32U4.
 * You are welcome to use whatever pin you want for triggering a ping, just be sure to change it from the default.
 * 
 * The input capture first looks for a rising edge, then a falling edge
 * The difference between the two is the pulse width, which is a direct measurement 
 * of the (round trip) timer counts to hear the echo.
 * 
 * But note that the timing is in timer counts, which must be converted to time.
 */

#include <Arduino.h>
#include "Romi32U4.h"

volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;

//define the states for the echo capture
enum PULSE_STATE {PLS_IDLE, PLS_WAITING_LOW, PLS_WAITING_HIGH, PLS_CAPTURED};

//and initialize to IDLE
volatile PULSE_STATE pulseState = PLS_IDLE;

//this may be most any pin, connect the pin to Trig on the sensor
const uint8_t trigPin = 14;

//for scheduling pings
uint32_t lastPing = 0;
const uint32_t PING_INTERVAL = 100; //ms

/*
 * Commands the ultrasonic to take a reading
 */

void CommandPing(int trigPin)
{
  cli(); //disable interrupts

  TIFR3 = 0x20; //clear any interrupt flag that might be there

  TIMSK3 |= 0x20; //enable the input capture interrupt
  TCCR3B |= 0xC0; //set to capture the rising edge on pin 13; enable noise cancel

  sei(); //re-enable interrupts

  //update the state and command a ping
  pulseState = PLS_WAITING_LOW;
  
  digitalWrite(trigPin, HIGH); //command a ping by bringing TRIG HIGH
  delayMicroseconds(10);      //we'll allow a delay here for convenience; it's only 10 us
  digitalWrite(trigPin, LOW);  //must bring the TRIG pin back LOW to get it to send a ping
}

Romi32U4Motors motors;
const float kp = 6.2;
const float ki = 0.1;
const float maxErrorSum = 10;
void standoff (float distance, float desireddistance){
  float error = distance - desireddistance;
  float sumOfErrors = sumOfErrors + error;
  if(sumOfErrors > maxErrorSum)
  {
    sumOfErrors = maxErrorSum;
  }
  if(sumOfErrors <-maxErrorSum)
  {
    sumOfErrors = -maxErrorSum;
  }
  float effort = error*kp;
  if(effort >270){effort = 270;}
  if(effort <-270){effort = -270;}
  motors.setEfforts(effort,effort);
}

void setup()
{
  Serial.begin(115200);
  while(!Serial) {} //you must open the Serial Monitor to get past this step!
  Serial.println("setup");

  noInterrupts(); //disable interupts while we mess with the control registers
  
  //sets timer 3 to normal mode (16-bit, fast counter)
  TCCR3A = 0; 
  
  interrupts(); //re-enable interrupts

  //note that the Arduino machinery has already set the prescaler elsewhere
  //so we'll print out the value of the register to figure out what it is
  Serial.print("TCCR3B = ");
  Serial.println(TCCR3B, HEX);

  pinMode(trigPin, OUTPUT);
  pinMode(13, INPUT); //explicitly make 13 an input, since it defaults to OUTPUT in Arduino World (LED)

  lastPing = millis();

  Serial.println("/setup");
}
int i =0;


int lastTime = 0;

float sharp_ir(){
  int time = millis();
  if(time - lastTime >= 100){
    float ADCReading = analogRead(A3);
    float voltageOut = (ADCReading * 5) / 1024;
    float sharpDistance = 19.7 / (voltageOut-0.342);
    lastTime = time;
    i++;
    return sharpDistance;
  }
}



float sumAve = 0;
float average(float values[5]){
  sumAve = values[0] + values[1] + values[2] + values[3] + values[4];
  float ave = sumAve/5;
  return ave;
}


float findMedian(float lastFive[5]){

float sorted[5] = {lastFive[0],-1,-1,-1,-1};

 for(int j=1; j<5; j++) {
    for(int k=0; k<5; k++) {
      if (lastFive[j] <= sorted[k]){
        for (int w=4; w>k; w--){
          sorted[w] = sorted[w-1];
        }
        sorted[k] = lastFive[j];
        break;
      }
    }
 }

return sorted[2];
}

float fiveValues[5] = {0,0,0,0,0};
void loop() 
{
  //schedule pings roughly every PING_INTERVAL milliseconds
  
  uint32_t currTime = millis();
  if((currTime - lastPing) >= PING_INTERVAL && pulseState == PLS_IDLE)
  {
    lastPing = currTime;
    CommandPing(trigPin); //command a ping
  }
  
  if(pulseState == PLS_CAPTURED) //we got an echo
  {
    //update the state to IDLE
    pulseState = PLS_IDLE;

    /*
     * Calculate the length of the pulse (in timer counts!). Note that we turn off
     * interrupts for a VERY short period so that there is no risk of the ISR changing
     * pulseEnd or pulseStart. The way the state machine works, this wouldn't 
     * really be a problem, but best practice is to ensure that no side effects can occur.
     */
    noInterrupts();
    uint16_t pulseLengthTimerCounts = pulseEnd - pulseStart;
    interrupts();

    
    //EDIT THIS LINE: convert pulseLengthTimerCounts, which is in timer counts, to time, in us
    //You'll need the clock frequency and the pre-scaler to convert timer counts to time
    
    uint32_t pulseLengthUS = pulseLengthTimerCounts * 4; //pulse length in us


    //EDIT THIS LINE AFTER YOU CALIBRATE THE SENSOR: put your formula in for converting us -> cm
    float ultraDivisor = 58.7;
    float ultraOffset = 15.5;
    float distancePulse = (pulseLengthUS + ultraOffset)/ ultraDivisor;    //distance in cm


    
    //replacing the five values to calculate based on
    fiveValues[0] = fiveValues[1];
    fiveValues[1] = fiveValues[2];
    fiveValues[2] = fiveValues[3];
    fiveValues[3] = fiveValues[4];
    fiveValues[4] = distancePulse;

    float processedData;
    //processedData = findMedian(fiveValues);
    processedData = average(fiveValues);

    standoff(processedData, 20);
    //Serial.print(millis());
    //Serial.print('\t');
    //Serial.print(pulseLengthTimerCounts);
    //Serial.print('\t');
    //Serial.print(pulseLengthUS);
    //Serial.print('\t');

    //"if" statement to just print 200 points
    //if(i<200)
    //{
    Serial.println(processedData);
    //i++;
    //}
    //Serial.print('\n');
    //Serial.print(processedData);
    //Serial.print("\t");
    //Serial.println(sharp_ir());
  }

  //for printing the sharpIR values
  //Serial.println(sharp_ir());
}

/*
 * ISR for input capture on pin 13. We can precisely capture the value of TIMER3
 * by setting TCCR3B to capture either a rising or falling edge. This ISR
 * then reads the captured value (stored in ICR3) and copies it to the appropriate
 * variable.
 */
ISR(TIMER3_CAPT_vect)
{
  if(pulseState == PLS_WAITING_LOW) //we're waiting for a rising edge
  {
    pulseStart = ICR3; //copy the input capture register (timer count)
    TCCR3B &= 0xBF;    //now set to capture falling edge on pin 13
    pulseState = PLS_WAITING_HIGH;
  }

  else if(pulseState == PLS_WAITING_HIGH) //waiting for the falling edge
  {
    pulseEnd = ICR3;
    pulseState = PLS_CAPTURED; //raise a flag to indicate that we have data
  }
}
