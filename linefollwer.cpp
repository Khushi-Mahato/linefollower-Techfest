#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <SparkFun_TB6612.h> // download the library first and run the code

bool isBlackLine = 1;
unsigned int lineThickness = 25;
unsigned int numSensors = 5;
bool brakeEnabled = 0;

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfSpeed = 120;     // 120
int currentSpeed = 50; // 30

float Kp = 0.04;
float Kd = 0.06;
float Ki = 0;

int onLine = 1;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorArray[7];
bool brakeFlag = 0;
String recordedPath = "";

void setup()
{
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  lineThickness = constrain(lineThickness, 10, 35);
}

void loop()
{
  while (digitalRead(11))
  {
  }
  delay(1000);
  calibrate();
  while (digitalRead(12))
  {
  }
  delay(1000);

  while (1)
  {
    readLine();
    if (currentSpeed < lfSpeed)
      currentSpeed++;
    if (onLine == 1)
    { // PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
      brakeFlag = 0;

      // Record the path
    }
    else
    {
      digitalWrite(13, LOW);
      if (error > 0)
      {
        if (brakeEnabled == 1 && brakeFlag == 0)
        {
          motor1.drive(0);
          motor2.drive(0);
          delay(30);
        }
        motor1.drive(-100);
        motor2.drive(150);
        brakeFlag = 1;

        // Record the path
      }
      else
      {
        if (brakeEnabled == 1 && brakeFlag == 0)
        {
          motor1.drive(0);
          motor2.drive(0);
          delay(30);
        }
        motor1.drive(150);
        motor2.drive(-100);
        brakeFlag = 1;

        // Record the path
      }
    }
  }
}

void linefollow()
{
  if (numSensors == 5)
  {
    error = (3 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 3 * sensorValue[5]);
  }
  if (lineThickness > 22)
  {
    error = error * -1;
  }
  if (isBlackLine)
  {
    error = error * -1;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // Adjust PIDvalue for prioritizing left turns
  if (error > 0)
  {
    PIDvalue *= 1.60; // Increase the weight for left turns 35
  }

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 255)
  {
    lsp = 255;
  }
  if (lsp < 0)
  {
    lsp = 0;
  }
  if (rsp > 255)
  {
    rsp = 255;
  }
  if (rsp < 0)
  {
    rsp = 0;
  }
  motor1.drive(lsp);
  motor2.drive(rsp);
  if (analogRead(A1) >= 900 && analogRead(A2) >= 900 && analogRead(A3) >= 900 && analogRead(A4) >= 900 && analogRead(A5) >= 900)
  {
    motor1.drive(0);
    motor2.drive(100);
    delay(400);

    if (analogRead(A1) >= 900 && analogRead(A2) >= 900 && analogRead(A3) >= 900 && analogRead(A4) >= 900 && analogRead(A5) >= 900)
    {
      motor1.drive(0);
      motor2.drive(0);

      delay(5000);
    }
  }
  //         else{
  //           continue;
  //         }
}

void calibrate()
{
  for (int i = 0; i < 6; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 10000; i++)
  {
    motor1.drive(70);
    motor2.drive(-70);

    for (int i = 0; i < 6; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for (int i = 0; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1.drive(0);
  motor2.drive(0);
}

void readLine()
{
  onLine = 0;
  if (numSensors == 5)
  {
    for (int i = 0; i < 7; i++)
    {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      if (isBlackLine == 1 && sensorValue[i] > 700)
        onLine = 1;
      if (isBlackLine == 0 && sensorValue[i] < 700)
        onLine = 1;
    }
  }
}

bool allSensorsOn()
{
  for (int i = 0; i < numSensors; i++)
  {
    if (sensorValue[i] < 900)
    {
      return false; // At least one sensor is not on the line
    }
  }
  return true; // All sensors are on the line
}