#include <Arduino.h>

/*
    https://github.com/pu2clr/RDA5807
    By Ricardo Lima Caratti, 2020.
*/
#include <RDA5807.h>
#include "LedControl.h"
#include "Wire.h"

RDA5807 rx;
int pinA = 2; // Connected to CLK on KY-040
int pinB = 3; // Connected to DT on KY-040
int encoderPosCount = 0;
int pinALast;
int aVal;
boolean bCW;

// max7219
byte brilho = 0;
#define N_DEV 1 // quantos max7219? só um.
// pinos fisicos atmega8  18.17.16
LedControl lc1 = LedControl(12, 11, 10, N_DEV);
/*
     DIN       -> MOSI    (Arduino output)
     CLK       -> SCK     (Arduino output)
     LOAD/#CS  -> SS      (Arduino output)
*/

// rda5807
int myfreq = 9250;
int myFreq_aux = 0;
bool myFreq_change = false;

///************ usado para gravar dados ds3231 ************
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ((val / 10 * 16) + (val % 10));
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ((val / 16 * 10) + (val % 16));
}
///********************************************************

// ds3231
#define DS3231_I2C_ADDRESS 0x68

void i2c_eeprom_write_byte(int deviceaddress, unsigned int eeaddress, byte data)
{
  int rdata = data;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
}

byte i2c_eeprom_read_byte(int deviceaddress, unsigned int eeaddress)
{
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress, 1);
  if (Wire.available())
    rdata = Wire.read();
  return rdata;
}

/**
 * Returns true if device found
 */
bool checkI2C()
{
  Wire.begin();
  byte error, address;
  int nDevices;
  Serial.println("I2C bus Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("\nI2C device found at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("\nUnknow error at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
    return false;
  }
  else
  {
    Serial.println("done\n");
    return true;
  }
}

void setup()
{

  Serial.begin(9600);
  while (!Serial)
    ;

  /* Rotary Encoder: */
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinALast = digitalRead(pinA);

  // max7219
  lc1.shutdown(0, false);
  int devices = lc1.getDeviceCount();
  brilho = i2c_eeprom_read_byte(0x57, 0x00); // pega o valor do brilho na memoria
  for (int address = 0; address < devices; address++)
  {
    /*The MAX72XX is in power-saving mode on startup*/
    lc1.shutdown(address, false);
    /* seta o brilho conforme memória */
    lc1.setIntensity(address, brilho);
    /* limpa o display */
    lc1.clearDisplay(address);
  }

  delay(500);

  if (!checkI2C())
  {
    Serial.println("\nCheck your circuit!");
    while (1)
      ;
  }

  rx.setup();
  rx.setVolume(8);

  delay(500);

  Serial.print("\nRSSI: ");
  Serial.print(rx.getRssi());

  // Mute test
  Serial.print("\nAfter 5s device will mute during 3s");
  delay(500);
  rx.setMute(true);
  delay(3000);
  rx.setMute(false);
  Serial.print("\nMute test has finished.");

  Serial.print("\nEstacao 99.3MHz");
  rx.setFrequency(myfreq);
  delay(1000);
}

void loop()
{

  aVal = digitalRead(pinA);
  if (aVal != pinALast)
  { // Means the knob is rotating
    // if the knob is rotating, we need to determine direction
    // We do that by reading pin B.
    if (digitalRead(pinB) != aVal)
    { // Means pin A Changed first - We're Rotating Clockwise
      encoderPosCount++;
      bCW = true;

      myfreq += 10;
      if (myfreq > 10750)
        myfreq = 10750;
      rx.setFrequency(myfreq);
      Serial.println(myfreq);
      myFreq_change = true;
    }
    else
    { // Otherwise B changed first and we're moving CCW
      bCW = false;
      encoderPosCount--;

      myfreq -= 10;
      if (myfreq < 8870)
        myfreq = 8870;

      rx.setFrequency(myfreq);
      Serial.println(myfreq);
      myFreq_change = true;
    }
    Serial.print("Rotated: ");
    if (bCW)
    {
      Serial.println("clockwise");
    }
    else
    {
      Serial.println("counterclockwise");
    }
    Serial.print("Encoder Position: ");
    Serial.println(encoderPosCount);
  }

  if (myFreq_change)
  {
    if (myfreq > 10000) // 107.5 =>10750
    {
      lc1.setDigit(0, 0, 1, false);
      lc1.setDigit(0, 1, 0, false);
      myFreq_aux = myfreq - 10000; // 0750
    }
    else // 81.3 => 8130
    {
      lc1.setRow(0, 0, 0x00);
      lc1.setDigit(0, 1, myfreq / 1000, false); // 8 <só o inteiro>
      myFreq_aux = myfreq % 1000;               // resto de 8130/1000 = 130
    }
    lc1.setDigit(0, 2, myFreq_aux / 100, true); // 1 <só inteiro>
    myFreq_aux = myFreq_aux % 100;              // resto de 130/100 = 30
    lc1.setDigit(0, 3, myFreq_aux / 10, false); // 30 / 10 = 3

    myFreq_change = false;
  }
  pinALast = aVal;
}