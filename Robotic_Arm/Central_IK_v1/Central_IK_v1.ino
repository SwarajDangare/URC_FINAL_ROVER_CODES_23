//#include <analogWrite.h>
#include <HardwareSerial.h>
#include "arm_IK.h"
#include "drive.h"

//---ESP32_PIN_CONFIG---
//Arm
#define dir_swivel 4
#define pwm_swivel 18
#define dir_link1 12
#define pwm_link1 27
#define dir_link2 16
#define pwm_link2 17

//Drive
#define Rdir 23
#define Ldir 32
#define Rpwm 19
#define Lpwm 33

//Limit Switch
#define L1 34
#define L2 35
#define L3 15
#define L4 14
#define Stow 5

// SERIAL
HardwareSerial SerialPort(0); // use UART0
HardwareSerial Sender(1);
const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Arm Pins
const int pwm_pin[] = {pwm_swivel, pwm_link1, pwm_link2, Rpwm, Lpwm};
const int dir_pin[] = {dir_swivel, dir_link1, dir_link2, Rdir, Ldir};
const int limit_pin[] = {L1, L2, L3, L4, Stow};

// Globals
long prevT = 0;
int changeMode = 0;
int r = 0;

// Arm Variables
int set_link[3];
int pwr[3];
int dir[3];
float linkLength1 = 39.8, linkLength2 = 37.5;

// Class instances
IK ik;
Drive drive;


void arm(int x, int y, int z, int Reset)
{
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  //Get Arm controls
  ik.arm_control(x, y, z, deltaT, Reset);
}

void gripper(int grip)
{
  switch (grip)
  {
    case 0: 
      Sender.write('0');
      //      Serial.println("0");
      break;

    case 1:
      Sender.write('1');
      //      Serial.println("1");
      break;

    case 2: 
      Sender.write('2');
      //      Serial.println("2");
      break;

    case 3: 
      Sender.write('3');
      //      Serial.println("3");
      break;

    case 4: 
      Sender.write('4');
      //      Serial.println("4");
      break;

    case 5: 
      Sender.write('5');
      //      Serial.println("5");
      break;

    case 6: 
      Sender.write('6');
      //      Serial.println("6");
      break;

    case 7: 
      Sender.write('7');
      //      Serial.println("7");
      break;

    default:
      Sender.write('0');
      //      Serial.println("7");
      break;
  }
}



void setup()
{
  Serial.begin(115200);
  SerialPort.begin(115200) ;   // Use default serial for debug output
  Sender.begin(115200, SERIAL_8N1, 0, 2);//(baud rate,protocol,Tx,Rx)

  // Limit Switch INPUT Pins
  for (int i = 0; i < 5; i++)
  {
    pinMode(limit_pin[i], INPUT);
  }

  // Set Drive Output Pins
  drive.setDrivePins(dir_pin[3], dir_pin[4], pwm_pin[3], pwm_pin[4]);
  ik.setArmPins(dir_pin[0], pwm_pin[0], dir_pin[1], pwm_pin[1], dir_pin[2], pwm_pin[2]);
  Serial.println("Sare pins set hogaye!");
  delay(100);

  ik.set_link_length(linkLength1, linkLength2);

  ik.set_pid();
  Serial.println("PID set");
  delay(10);

  ik.set_IMU();
  Serial.println("IMUs connected!!!");

  ik.set_init_coordinates();

}

void loop()
{
  if (SerialPort.available())
  {
    // read the data into the buffer
    while (SerialPort.available())
    {
      rxBuffer[bufferIndex] = (char)SerialPort.read();
      bufferIndex++;
      // Make sure we don't overflow the buffer
      if (bufferIndex >= BUFFER_SIZE)
        bufferIndex = 0;
    }
    //    Serial.println(rxBuffer);

    // Find the positions of the "M", "X", "Y", "P", "Q", "A", "S", "R", "D" and "E" characters in the buffer
    char *M_index = strchr(rxBuffer, 'M'); // gear
    char *x_index = strchr(rxBuffer, 'X'); // drive x
    char *y_index = strchr(rxBuffer, 'Y'); // drive y
    char *P_index = strchr(rxBuffer, 'P'); // arm X
    char *Q_index = strchr(rxBuffer, 'Q'); // arm Y
    char *A_index = strchr(rxBuffer, 'A'); // gripper
    char *S_index = strchr(rxBuffer, 'S'); // arm Z
    char *R_index = strchr(rxBuffer, 'R'); // Reset
    char *D_index = strchr(rxBuffer, 'D'); // Mode
    char *E_index = strchr(rxBuffer, 'E'); // End

    if (M_index != NULL && x_index != NULL && y_index != NULL && P_index != NULL && Q_index != NULL && A_index != NULL && S_index != NULL && R_index != NULL && D_index != NULL && E_index != NULL)
    {
      // Extract the values from the packet
      char m = *(M_index + 1);
      int M = m - '0';
      int x = atoi(x_index + 1);
      int y = atoi(y_index + 1);
      int X = atoi(P_index + 1);
      if (abs(X) < 3)
      {
        X = 0;
      }
      int Y = atoi(Q_index + 1);
      if (abs(Y) < 3)
      {
        Y = 0;
      }
      int grip = atoi(A_index + 1);
      int Z = atoi(S_index + 1);
      if (abs(Z) < 3)
      {
        Z = 0;
      }
      int Reset = atoi(R_index + 1);
      int Mode = atoi(D_index + 1);
      drive.MotorCode(x, y, M);
      if (Mode == 1)
      {
        if (changeMode == 0)
        {
          ik.Stop();
          changeMode = 1;
        }
        drive.MotorCode(x, y, M);
      }
      else
      {
        if (changeMode == 1)
        {
          ik.drive_rm();
          changeMode = 0;
        }
        drive.MotorCode(x, y, M);
        arm(X, Y, Z, Reset);
        gripper(grip);
      }
      delay(10);

    }
    else
    {
      Serial.println("Invalid Packet received");
    }
  }
  else
  {
    ik.Stop();
    drive.Stop();
//    r++;
//    if (r == 1)
//    {
//      delay(10);
//      ESP.restart();
//      r = 0;
//    }
  }
  bufferIndex = 0;
}
