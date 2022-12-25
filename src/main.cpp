#include <Arduino.h>

/*
    https://github.com/pu2clr/RDA5807
    By Ricardo Lima Caratti, 2020.
*/
#include <RDA5807.h>

RDA5807 rx;

int pinA = 2; // Connected to CLK on KY-040
int pinB = 3; // Connected to DT on KY-040
int encoderPosCount = 0;
int pinALast;
int aVal;
boolean bCW;
int myfreq = 9250;
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
  /* Read Pin A
  Whatever state it's in will reflect the last position
  */
  pinALast = digitalRead(pinA);

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
  /*
  Serial.print("\nEstacao 106.5MHz");
  rx.setFrequency(9930); // The frequency you want to select in MHz multiplied by 100.

  Serial.print("\nCurrent Channel: ");
  Serial.print(rx.getRealChannel());
  delay(500);

  Serial.print("\nReal Frequency.: ");
  Serial.print(rx.getRealFrequency());
*/
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

  /*
  // Seek test
  Serial.print("\nSeek station");
  for (int i = 0; i < 10; i++ ) {
    rx.seek(1,0);
    Serial.print("\nReal Frequency.: ");
    Serial.print(rx.getRealFrequency());
    delay(5000);
  }
  */
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
    }
    else
    { // Otherwise B changed first and we're moving CCW
      bCW = false;
      encoderPosCount--;

      myfreq -= 10;
      if (myfreq < 8870)
      {
        myfreq = 8870;
      }
      rx.setFrequency(myfreq);
      Serial.println(myfreq);
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
  pinALast = aVal;
}