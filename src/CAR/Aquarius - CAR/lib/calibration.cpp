#include <QTRSensors.h>
#include <EEPROM.h>

#define NUM_SENSORS   5     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low

#define ADDR_CALIBRATED_MINIMUM_ON 0
#define ADDR_CALIBRATED_MAXIMUM_ON 100

byte ir1 = 26;
byte ir2 = 27;
byte ir3 = 28;
byte ir4 = 29;
byte ir5 = 30;
QTRSensors qtr;

unsigned int sensorValues[NUM_SENSORS];



void setup()
{
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ir1, ir2, ir3, ir4, ir5}, IR_QRT_COUNT);
  Serial.begin(9600);
  Serial.println("QTR Sensor Calibration. (C)alibrate, (R)ead, (S)tore to EEPROM, R(E)call from EEPROM");
}


void loop()
{

  if (Serial.available())
  {
    char ch = Serial.read();

    if (ch == 'c' || ch == 'C')
    {
      calibrateQTR();
    }
    else if (ch == 'r' || ch == 'R')
    {
      readQTR();
    }
    else if (ch == 's' || ch == 'S')
    {
      storeQTR();
    }
    else if (ch == 'e' || ch == 'E')
    {
      recallQTR();
    }
  }

}

void calibrateQTR()
{
  Serial.println();
  Serial.println("Beginning Calibration Process...");

  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
 
  Serial.println("Calibration Complete");

}

void readQTR()
{
  Serial.println();
  Serial.println("Reading Calibration Data...");

   for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  
}

void storeQTR()
{
  Serial.println();
  Serial.println("Storing Calibration Data into EEPROM...");

  EEPROM.writeBlock<unsigned int>(ADDR_CALIBRATED_MINIMUM_ON, qtrrc.calibratedMinimumOn, 8);
  EEPROM.writeBlock<unsigned int>(ADDR_CALIBRATED_MINIMUM_ON, qtrrc.calibratedMaximumOn, 8);

  Serial.println("EEPROM Storage Complete");
}

void recallQTR()
{
  Serial.println();
  Serial.println("Recalling Calibration Data from EEPROM...");

  qtrrc.calibrate(); 
  EEPROM.readBlock<unsigned int>(ADDR_CALIBRATED_MINIMUM_ON, qtrrc.calibratedMinimumOn, 8);
  EEPROM.readBlock<unsigned int>(ADDR_CALIBRATED_MAXIMUM_ON, qtrrc.calibratedMaximumOn, 8);

  Serial.println("EEPROM Recall Complete");
}