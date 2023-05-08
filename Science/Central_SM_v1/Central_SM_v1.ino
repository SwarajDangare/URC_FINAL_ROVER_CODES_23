#include <HardwareSerial.h>
#include "drive.h"


//---ESP32_PIN_CONFIG---
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
#define Extra 5

// SERIAL
HardwareSerial SerialPort(0); // use UART0
HardwareSerial Sender(1);
const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Pins
const int pwm_pin[] = {Rpwm, Lpwm};
const int dir_pin[] = {Rdir, Ldir};
const int limit_pin[] = {L1, L2, L3, L4, Extra};

// Class instances
Drive drive;

int x = 0, y = 0;
float M = 0;
char c;
char data[16] = "000000000000000";

void sm(int sm, int reset)
{
  if (reset == 1)
  {
    Sender.write('A');
    delay(100);
    ESP.restart();
  }
  switch (sm)
  {
    case 0: // safety
      Sender.write('0');
      //      Serial.println("0");
      break;

    case 1: // pump
      Sender.write('1');
      //      Serial.println("1");
      break;

    case 2: // vacumm
      Sender.write('2');
      //      Serial.println("2");
      break;

    case 3: // stepper rotate
      Sender.write('3');
      break;

    case 4: // stepper direction toggle
      Sender.write('4');
      break;

    case 5: // auger down with rotation
      Sender.write('5');
      break;

    case 6: // auger up
      Sender.write('6');
      break;

    case 7: // auger rotation for deposition
      Sender.write('7');
      break;

    case 8: // sensor suite up
      Sender.write('8');
      break;

    case 9: // sensor suite down
      Sender.write('9');
      break;

    default:
      Sender.write('0');
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
  drive.setDrivePins(dir_pin[0], dir_pin[1], pwm_pin[0], pwm_pin[1]);
  Serial.println("Sare pins set hogaye!");
  delay(100);
  
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

    // Find the positions of the "M","X", "Y", "S" ,"R" and "E" characters in the buffer
    char *M_index = strchr(rxBuffer, 'M');
    char *x_index = strchr(rxBuffer, 'X');
    char *y_index = strchr(rxBuffer, 'Y');
    char *S_index = strchr(rxBuffer, 'S');
    char *R_index = strchr(rxBuffer, 'R');
    char *D_index = strchr(rxBuffer, 'D'); // Mode
    char *E_index = strchr(rxBuffer, 'E');

    if (M_index != NULL && x_index != NULL && y_index != NULL && S_index != NULL && R_index != NULL && D_index != NULL && E_index != NULL)
    {
      // Extract the values from the packet
      char m = *(M_index + 1);
      int M = m - '0';
      int x = atoi(x_index + 1);
      int y = atoi(y_index + 1);
      int s = atoi(S_index + 1);
      int r = atoi(R_index + 1);
      int Mode = atoi(D_index + 1);
      drive.MotorCode(x, y, M);
      if (Mode == 0)
      {
        drive.MotorCode(x, y, M);
        sm(s, r);
      }
      else
      {
        drive.MotorCode(x, y, M);
      }
    }
    else
    {
      Serial.println("Invalid Packet received");
    }
  }
  else
  {
    drive.Stop();
    Sender.write('0');
  }
  bufferIndex = 0;
}
