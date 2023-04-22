//#include <HardwareSerial.h>
#include "pid.h"
//--ESP32-PINS--
//ENCODER
#define Left_EA 4
#define Left_EB 5
#define Right_EA 19
#define Right_EB 13
#define Grip_EA 14
#define Grip_EB 15
//GRIPPER MOTORS
#define Left_PWM 27
#define Left_DIR 32
#define Right_PWM 33
#define Right_DIR 23
#define Grip_PWM 17
#define Grip_DIR 16

const int enca[] = {Left_EA, Right_EA, Grip_EA};
const int encb[] = {Left_EB, Right_EB, Grip_EB};
const int pwm_pin[] = {Left_PWM , Right_PWM , Grip_PWM };
const int dir_pin[] = {Left_DIR , Right_DIR , Grip_DIR };

//--SERIAL--
HardwareSerial SerialPort(0); // use UART0
const int BUFFER_SIZE = 5;
char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;

//--NUMBER-OF-MOTORS--
#define NMOTORS 3
int target[NMOTORS];

//--MOTOR-PWM--
const int freq = 5000;
const int channel[NMOTORS] = {0, 1, 2};
const int resolution = 8;

// Globals
long prevT = 0;
volatile int posi[] = {0, 0, 0};

// PID class instances
PID pid[NMOTORS];

void get_Grip_Cmd(char grip) {
  switch (grip)
  {
    case '0': // safety
      Serial.println("0");
      target[0] = posi[0];
      target[1] = posi[1];
      target[2] = posi[2];
      break;

    case '1': // safety
      Serial.println("Pitch Down");

      target[0] -= 2;
      target[1] += 2;

      break;

    case '2': // safety
      Serial.println("Pitch Up");
      target[0] += 2;
      target[1] -= 2;

      break;

    case '3': // safety
      Serial.println("Roll Right");
      target[0] += 2;
      target[1] += 2;

      break;

    case '4': // safety
      Serial.println("Roll Left");

      target[0] -= 2;
      target[1] -= 2;
      break;

    case '5': // safety
      Serial.println("Gripperr Close");
      target[2] -= 20;
      break;

    case '6': // safety
      Serial.println("Gripperr Open");
      target[2] += 20;
      break;

    case '7': // safety
      Serial.println("Reset");
      posi[0] = 0;
      posi[1] = 0;
      posi[2] = 0;
      Stop();
      break;

    default:
      Serial.println("0");
      target[0] = posi[0];
      target[1] = posi[1];
      target[2] = posi[2];
      Stop();
      break;
  }
}

void set_Gripper()
{
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos[NMOTORS];
  noInterrupts(); // disable interrupts temporarily while reading
  for (int k = 0; k < NMOTORS; k++) {
    pos[k] = posi[k];
  }
  interrupts(); // turn interrupts back on

  // loop through the motors
  for (int k = 0; k < NMOTORS; k++) {
    int pwr, dir;
    // evaluate the control signal
    pid[k].getpid(pos[k], target[k], deltaT, pwr, dir);
    // signal the motor
    setMotor(dir, pwr, k, dir_pin[k]);
  }
  Serial.println();

  for (int k = 0; k < NMOTORS; k++)
  {
    Serial.print("Target: ");
    Serial.print(target[k]);
    Serial.print(" Pos: ");
    Serial.println(pos[k]);
  }
  //  Serial.println();
}


void setMotor(int dir, int pwmVal, int pwm_channel, int dir_pin)
{
  ledcWrite(channel[pwm_channel], pwmVal);
  digitalWrite(dir_pin, dir);
  Serial.print("Dir: ");
  Serial.print(dir);
  Serial.print(" PWM: ");
  Serial.print(pwmVal);
}

void Stop()
{
  ledcWrite(channel[0], 0);
  ledcWrite(channel[1], 0);
  ledcWrite(channel[2], 0);
  Serial.println("Stop");
}

template <int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if (b > 0) {
    posi[j]++;
  }
  else
  {
    posi[j]--;
  }
}

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200) ;   // Use default serial for debug output
  delay(500);

  for (int k = 0; k < NMOTORS; k++)
  {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(dir_pin[k], OUTPUT);
    ledcSetup(channel[k], freq, resolution);
    ledcAttachPin(pwm_pin[k], channel[k]);

    ledcWrite(channel[k], 0);
  }
  Serial.println("Sare pins set hogaye!");
  delay(100);

  pid[0].setParams(8, 0, 0, 230, 10, 1);
  pid[1].setParams(8, 0, 0, 255, 10, 1);
  pid[2].setParams(30, 0, 0, 255, 10, 1);
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);

  Serial.println("PID set");
  delay(10);

  //  Serial.println("target pos");
}

void loop()
{
  if (SerialPort.available())
  {
    // read the data into the buffer
    //    while (SerialPort.available())
    //    {
    //      rxBuffer[bufferIndex] = (char)SerialPort.read();
    //      bufferIndex++;
    //      // Make sure we don't overflow the buffer
    //      if (bufferIndex >= BUFFER_SIZE)
    //        bufferIndex = 0;
    //    }
    while (SerialPort.available())
    {
      char RxdChar = SerialPort.read();
      Serial.println(RxdChar);
      get_Grip_Cmd(RxdChar);
      set_Gripper();
      Serial.println("");
    }
    //    Serial.println(rxBuffer);
  }
  //  else
  //  {
  //    Stop();
  //  }
  bufferIndex = 0;
}
