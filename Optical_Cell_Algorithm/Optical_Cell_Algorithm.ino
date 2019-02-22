
/**
   Creating a rubidium optical cell that keeps a constant tempature and creates a cold finger that allows condensation
   away from the optical lenses
   @AUTHOR James Deromedi

   PID setup (input, output, setpoint, Kp, Ki, Kd, direction)
   Kp -Determines how aggressively PID reacts to the current error
   Ki -Determines how aggressively PID reacts to error over time
   Kd -Determines how aggressively PID reacts to change in error
   Direction -Does the PID increase or decrease depending (DIRECT: output decreased, REVERSE: output increased )
*/

#include <Adafruit_MAX31865.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

const float RREF = 4300.0;                //Value of reference resistor
const float RNOMINAL = 1000.0;                      //Value of RTd at 0 celcius
const int DAC = A14;
const int POT3_3 = A1;
const int HEATINGLED = 8;

double hotSetPoint, hotInput, hotOutput;
//double coldSetPoint, coldInput, coldOutput;
double kP = 400;
double kI = 300;
double kD = 30;
bool heatedUp = false;
char hotTempBuffer[4] = {'0', '0', '0', '0'};

Adafruit_AlphaNum4 hotTempAlpha = Adafruit_AlphaNum4();

Adafruit_MAX31865 hotSensor = Adafruit_MAX31865(10);  //Hardware SPI, CS on 10
Adafruit_MAX31865 coldSensor = Adafruit_MAX31865(9);  //Hardware SPI, CS on 9

PID hotPID (&hotInput, &hotOutput, &hotSetPoint, kP, kI, kD, DIRECT);   //PID object for the hot TEC
//PID coldPID (&coldInput, &coldOutput, &coldSetPoint, 2, 5, 1, DIRECT);   //PID object for the cold TEC


void setup() {
  Serial.begin(115200);
  while (!Serial);
  hotSensor.begin(MAX31865_2WIRE);
  //coldSensor.begin(MAX31865_2WIRE);

  hotSetPoint = 35.0;                //Target temp for hot sensor
  //coldSetPoint = 95;                //Target temp for cold sensor
  hotPID.SetMode(AUTOMATIC);        //PID automatically adjusts output smoothly when setpoint changes
  hotPID.SetSampleTime(10);         //Set PID sample time to 10ms
  hotPID.SetOutputLimits(0, 4095);  //Set range of PID output to DAC output
  //coldPID.SetMode(AUTOMATIC);
  //coldPID.SetSampleTime(10);
  //coldPID.SetOutputLimits(0, 4095);

  hotTempAlpha.begin(0x70);
  hotTempAlpha.setBrightness(6);

  analogWriteResolution(12);        //Allows for more accurate output to the DAC
  pinMode(HEATINGLED, OUTPUT);      //Attach 100 Ohm to ground
  digitalWrite(HEATINGLED, HIGH);
}

void loop() {
  faultCheck(); //Checks for faults

  float hotTemp = hotSensor.temperature(RNOMINAL, RREF);       //Tempature of hot sensor
  float coldTemp = coldSensor.temperature(RNOMINAL, RREF);     //Tempature of cold sensor

  startHeating(hotTemp);

  //printAllValues(hotInput, coldInput);          //Used for testing
  printPlotter(hotTemp, coldTemp);              //Used for printing to Serial.plotter
  alphaPrint(hotTemp);
}//END LOOP

//====================================================================================
/**
   Assigns buffer to print to alpha display
*/
void alphaPrint(float temp) {
  numberSeperation(temp);
  
  hotTempAlpha.writeDigitAscii(0, hotTempBuffer[0]);
  hotTempAlpha.writeDigitAscii(1, hotTempBuffer[1]);
  hotTempAlpha.writeDigitAscii(2, hotTempBuffer[2]);
  hotTempAlpha.writeDigitAscii(3, hotTempBuffer[3]);

  hotTempAlpha.writeDisplay();
}//END ALPHAPRINT

//====================================================================================
/**
   Controls warming up the device and than activating the PID when hot
*/
void numberSeperation(float hot) {
  hotTempBuffer[3] = (int(hot) % 10) + 48;
  hotTempBuffer[2] = ((int(hot) / 10) % 10) + 48;
  hotTempBuffer[1] = (((int(hot) / 10) % 10) % 10) + 48;
  hotTempBuffer[0] = ((((int(hot) / 10) % 10) % 10) % 10) + 48;
}//END NUMBERSEPERATION

//====================================================================================
/**
   Controls warming up the device and than activating the PID when hot
*/
void startHeating(float hotTempeture) {
  hotInput = hotTempeture;
  if (!heatedUp) {                  //If not heated up yet, run at full power
    if (hotTempeture < 35.0) {
      analogWrite(DAC, 4095);
      digitalWrite(HEATINGLED, HIGH);   //Turns on LED
    } else {
      heatedUp = true;
      digitalWrite(HEATINGLED, LOW);   //Turns off LED
    }
  } else {                             //If heated up, run PID
    hotPID.Compute();
  }
  analogWrite(DAC, hotOutput);
}

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
  //  Serial.println("Cold PID Values");
  //  Serial.print("/t --Kp:"); Serial.println(coldPID.GetKp());
  //  Serial.print("/t --Ki:"); Serial.println(coldPID.GetKi());
  //  Serial.print("/t --Kd:"); Serial.println(coldPID.GetKd());
  //  Serial.print("/t --Mode:"); Serial.println(coldPID.GetMode());
  //  Serial.print("/t --Direction:"); Serial.println(coldPID.GetDirection());
  //  Serial.println("");
}//END PRINTALLVALUES

//====================================================================================
/**
   Prints the tempature (celcius) of the RTDs
   @PARAM float hotTempature: The converted hot RTD value into celcius
   @PARAM float coldTempature: The converted cold RTD value into celcius
   @RETURN None
*/
void printPlotter(float hotTempature, float coldTempature) {
  Serial.println(hotTempature);
  //Serial.print(",");
  //Serial.print(coldTempature);
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

