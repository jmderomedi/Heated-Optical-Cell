/**
   Creating a heated container for a rubidium optical cell that keeps a constant tempature and creates a cold finger that allows condensation
   away from the optical lenses
   @AUTHOR James Deromedi

   PID setup (input, output, setpoint, Kp, Ki, 0Kd, direction)
   Kp -Determines how aggressively PID reacts to the current error
   Ki -Determines how aggressively PID reacts to error over time
   Kd -Determines how aggressively PID reacts to change in error
   Direction -Does the PID increase or decrease depending (DIRECT: output decreased, REVERSE: output increased )
*/

#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <SFE_MicroOLED.h>
#include <SavLayFilter.h>

const float RREF = 4300.0;         //Value of reference resistor
const float RNOMINAL = 1000.0;     //Value of RTd at 0 celcius
const int HOTDAC = A22;
const int COLDDAC = A21;
const int KPPIN = A16;
const int KIPIN = A15;
const int KDPIN = A14;
const int SETPOINTPIN = A17;
const int HEATINGLED = 23;
const int PIN_RESET = 16;
const int PIN_DC = 17;
const int PIN_CS = 15;

double hotSetPoint, hotInput, hotOutput;
double coldSetPoint, coldInput, coldOutput;

double hotKP = 1800.0; double coldKP = 4096.0;
double hotKI = .0330;  double coldKI = 100.0;
double hotKD = 505.4; double coldKD = 4096.0;

double hotTemperature;
int convoluteNum = 0;
int windowSize = 5;

Adafruit_MAX31865 hotSensor = Adafruit_MAX31865(10);  //Hardware SPI, CS on 10
Adafruit_MAX31865 coldSensor = Adafruit_MAX31865(9);  //Hardware SPI, CS on 9

PID hotPID (&hotInput, &hotOutput, &hotSetPoint, hotKP, hotKI, hotKD, DIRECT);   //PID object for the hot TEC
PID coldPID (&coldInput, &coldOutput, &coldSetPoint, coldKP, coldKI, coldKD, DIRECT);   //PID object for the cold TEC
SavLayFilter filter (&hotTemperature, convoluteNum, windowSize);

MicroOLED oled(PIN_RESET, PIN_DC, PIN_CS);

void setup() {
  Serial.begin(115200);
  hotSensor.begin(MAX31865_2WIRE);
  coldSensor.begin(MAX31865_2WIRE);
  hotSetPoint = 30.0;                //Target temp for hot sensor
  coldSetPoint = 29.0;               //Target temp for cold sensor

  hotPID.SetMode(AUTOMATIC);        //PID automatically adjusts output smoothly when setpoint changes
  hotPID.SetSampleTime(50);         //Set PID sample time to 10ms
  hotPID.SetOutputLimits(0, 4095);  //Set range of PID output to DAC output
  coldPID.SetMode(AUTOMATIC);
  coldPID.SetSampleTime(50);
  coldPID.SetOutputLimits(0, 4095);

  analogWriteResolution(12);        //Allows for more accurate output to the DAC
  pinMode(HOTDAC, OUTPUT);
  pinMode(COLDDAC, OUTPUT);
  pinMode(HEATINGLED, OUTPUT);      //Attach 100 Ohm to ground

  oled.begin();
  oled.clear(ALL);
  oled.display();
  delay(1000);     // Delay 1000 ms
  oled.clear(PAGE); // Clear the buffer.
  oled.setFontType(0);
}


void loop() {
  faultCheck(); //Checks for faults

  hotInput = hotSensor.temperature(RNOMINAL, RREF);
  //coldInput = coldSensor.temperature(RNOMINAL, RREF);

  tuning(false);                                   //Allows for dynamic changes of the PID algorithm
  startHeating();                               //Calls the PID algorithm
  printPlotter(hotInput, coldInput);            //Used for printing to Serial.plotter
  LEDControl(hotInput, coldInput);                                 //Just controls LEDs



}//END LOOP

//====================================================================================
/**
   TODO: Add ability to change to second PID algorithm without repushing to Teensy
         Figure out why the pots have such a bouncy signal

   Reads the different pots and maps them to an appropriate range for each parameter
   Outputs the values to a OLED screen the values of the POT to be used for tuning
   @PARAM None
   @RETURN None
*/
void tuning(bool tune) {
  if (tune) {
    float kPInput = map(analogRead(KPPIN), 0, 1023, 0, 8190);
    float kIInput = map(analogRead(KIPIN), 0, 1023, 0, 4095);
    float kDInput = map(analogRead(KDPIN), 0, 1023, 0, 200);
    hotSetPoint = map(analogRead(SETPOINTPIN), 0, 1023, 20, 50);

    hotPID.SetTunings(kPInput, kIInput, kDInput);                   //Setting the new parameters

    //Writes the left side of the screen
    oled.clear(PAGE);
    oled.setCursor(0, 0);
    oled.print("Set:");
    oled.setCursor(0, 10);
    oled.print("kP:");
    oled.setCursor(0, 20);
    oled.print("kI:");
    oled.setCursor(0, 30);
    oled.print("kD:");

    //Writes the changing varaiables
    oled.setCursor(25, 0);
    oled.print(int(hotSetPoint));
    oled.setCursor(18, 10);
    oled.print(int(kPInput));
    oled.setCursor(18, 20);
    oled.print(int(kIInput));
    oled.setCursor(18, 30);
    oled.print(int(kDInput));

    oled.setCursor(33, 40);
    oled.print(hotInput);
    oled.display();
  } else {
    oled.clear(PAGE);
    oled.setCursor(33, 40);
    oled.print(hotInput);
    oled.display();
  }
}

//====================================================================================
/**
   Controls calling the PID algorithm, while the Compute() automatically controls when it is run
   @PARAM None
   @RETURN None
*/
void startHeating() {
  hotPID.Compute();
  //coldPID.Compute();

  analogWrite(HOTDAC, hotOutput);
  //analogWrite(COLDDAC, coldOutput);
}//END STARTHEATING

//====================================================================================
/**
   Controls LEDS
   @PARAM None
   @RETURN None
*/
void LEDControl(float hotTemp, float coldTemp) {
  if (hotTemp <= (hotSetPoint + 0.15) && hotTemp >= (hotSetPoint - 0.15)) {
    digitalWrite(HEATINGLED, HIGH);                //Turns on LED
  } else {
    digitalWrite(HEATINGLED, LOW);                 //Turns off LED
  }
  if (coldTemp < 29.1 && coldTemp > 28.9) {
    //TODO: Add second LED for cold temp
  }
}//END LEDCONTROL

//====================================================================================
/**
   Prints all values related to both tempatures and PID
   @PARAM float hotTempature: The converted hot RTD value into celcius
   @PARAM float coldTempature: The converted cold RTD value into celcius
   @RETURN None
*/
void printAllValues(float hotTempature, float coldTempature) {
  Serial.println("===========================================================");
  Serial.print("Hot temp: "); Serial.print(hotTempature, 8); Serial.print("; Cold temp: "); Serial.println(coldTempature, 8);
  Serial.println("");
  Serial.println("Hot PID Values");
  Serial.print("/t --Kp:"); Serial.println(hotPID.GetKp());
  Serial.print("/t --Ki:"); Serial.println(hotPID.GetKi());
  Serial.print("/t --Kd:"); Serial.println(hotPID.GetKd());
  Serial.print("/t --Mode:"); Serial.println(hotPID.GetMode());
  Serial.print("/t --Direction:"); Serial.println(hotPID.GetDirection());
  Serial.println("");
  Serial.println("Cold PID Values");
  Serial.print("/t --Kp:"); Serial.println(coldPID.GetKp());
  Serial.print("/t --Ki:"); Serial.println(coldPID.GetKi());
  Serial.print("/t --Kd:"); Serial.println(coldPID.GetKd());
  Serial.print("/t --Mode:"); Serial.println(coldPID.GetMode());
  Serial.print("/t --Direction:"); Serial.println(coldPID.GetDirection());
  Serial.println("");
}//END PRINTALLVALUES

//====================================================================================
/**
   Prints the tempature (celcius) of the RTDs
   @PARAM float hotTempature: The converted hot RTD value into celcius
   @PARAM float coldTempature: The converted cold RTD value into celcius
   @RETURN None
*/
void printPlotter(float hotTempature, float coldTempature) {
  Serial.print(hotSetPoint);
  Serial.print(",");
  //  Serial.print(coldSetPoint);
  //  Serial.print(",");
  //Serial.println(coldTempature);
  //  Serial.print(",");
  Serial.println(hotTempature);

}//END PRINTVALUES

//====================================================================================
/**
   Checks the RTDs for any faults and prints to serial if there is any
   @PARAM uint8_T hotFaults: The 8 bit fault that occured on the hot sensor
   @PARAM uint8_T coldFaults: The 8 bit fault that occured on the cold sensor
   @RETURN None
*/
void faultCheck() {
  uint8_t hotFaults = hotSensor.readFault();
  uint8_t coldFaults = coldSensor.readFault();
  if (hotFaults) {
    Serial.print("Fault on hot sensor 0x"); Serial.println(hotFaults, HEX);

    if (hotFaults & MAX31865_FAULT_HIGHTHRESH) { //RTD is reading max value, check wiring
      Serial.println("RTD High Threshold");
    }
    if (hotFaults & MAX31865_FAULT_LOWTHRESH) {  //RTD is rading lowest value, check RTD if it is broken
      Serial.println("RTD Low Threshold");
    }
    if (hotFaults & MAX31865_FAULT_REFINLOW) {   //Check MISO/MOSI wiring
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (hotFaults & MAX31865_FAULT_REFINHIGH) {  //Check MISO/MOSI wiring
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (hotFaults & MAX31865_FAULT_RTDINLOW) {   //Check MISO/MOSI wiring
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (hotFaults & MAX31865_FAULT_OVUV) {       //Check power supply wiring
      Serial.println("Under/Over voltage");
    }
    hotSensor.clearFault();
  }
  if (coldFaults) {
    Serial.print("Fault on cold sensor 0x"); Serial.println(coldFaults, HEX);
    if (coldFaults & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (coldFaults & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (coldFaults & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (coldFaults & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (coldFaults & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (coldFaults & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    coldSensor.clearFault();
  }

}//END CHECKFAULTS

