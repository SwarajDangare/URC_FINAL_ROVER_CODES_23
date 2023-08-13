#include "pid.h"
#include "imu.h"

class IK
{
  private:
    int dir_link1, pwm_link1, dir_link2, pwm_link2, dir_swivel, pwm_swivel;
    int limit_1, limit_2, limit_3, limit_4, limit_5;
    int pwmswl, dirswl, pwm1, dir1, pwm2, dir2;
    int freq, L1channel, L2channel, Swchannel, resolution;

    //final angles
    float set_link[3];
    //final coordinates
    float set_X, set_Y, set_Z;
    //current coordinates
    float x, y, z;
    //current angles
    float pitchsw = 0, pitch1 = 0, pitch2 = 0;
    float jointangle0, jointangle1, jointangle2, angle2;
    //link length
    float L1, L2;
    float max_reach = 0;
    float min_reach = 0;
    //ik variabls
    float b = 0, a = 0;

    // PID class instances
    PID arm_pid[3]; //swl,L1,L2

  public:
    IK()
    {
      freq = 8000;
      L1channel = 2;
      L2channel = 3;
      Swchannel = 4;
      resolution = 8;
    }

    void setArmPins( int dir_sw, int pwm_sw, int dir_1, int pwm_1, int dir_2, int pwm_2)
    {
      dir_link1 = dir_1;
      pwm_link1 = pwm_1;
      dir_link2 = dir_2;
      pwm_link2 = pwm_2;
      dir_swivel = dir_sw;
      pwm_swivel = pwm_sw;

      ledcSetup(L1channel, freq, resolution);
      ledcSetup(L2channel, freq, resolution);
      ledcSetup(Swchannel, freq, resolution);
      ledcAttachPin(pwm_link1, L1channel);
      ledcAttachPin(pwm_link2, L2channel);
      ledcAttachPin(pwm_swivel, Swchannel);
      pinMode(dir_link1, OUTPUT);
      pinMode(dir_link2, OUTPUT);
      pinMode(dir_swivel, OUTPUT);
    }
    void setLimitPins( int l1, int l2, int l3, int l4, int l5)
    {
      limit_1 = l1;
      limit_2 = l2;
      limit_3 = l3;
      limit_4 = l4;
      limit_5 = l5;

      pinMode(limit_1, INPUT);
      pinMode(limit_2, INPUT);
      pinMode(limit_3, INPUT);
      pinMode(limit_4, INPUT);
      pinMode(limit_5, INPUT);

    }

    void readLimit()
    {
      set_link[0] = 0;
      set_link[1] = 90;
      set_link[2] = 0;
      Serial.println("LIMIT HIT!!!!");
    }

    void set_pid(void)
    {
      //                   p,    d,     i,     max,min,hold
      arm_pid[0].setParams(15, 0.0000, 0.0010, 255, 25, 1); //swl
      arm_pid[1].setParams(25, 0.0000, 0.0010, 255, 25, 1); //L1
      arm_pid[2].setParams(30, 0.0000, 0.0010, 255, 25, 1); //L2
    }

    void set_IMU(void)
    {
      //---BNO055-1---
      setBNO0551();
      delay(500);

      //---BNO055-2---
      setBNO0552();
      delay(500);
    }

    void set_link_length(float l1, float l2)
    {
      L1 = l1;
      L2 = l2;
      max_reach = L1 + L2 - 7;
      min_reach = L1 - L2 * cos(radians(35)) + 5;
    }

    void set_init_coordinates(void)
    {
      //      set_link[0] = zBNO0552();
      //      set_link[1] = yBNO0551();
      //      set_link[2] = yBNO0552();

      pitchsw = zBNO0552();
      jointangle0 = radians(pitchsw);
      pitch1 = yBNO0551();
      jointangle1 = radians(pitch1);
      pitch2 = yBNO0552();
      angle2 = radians(pitch2);
      jointangle2 = jointangle1 - angle2;

      get_corrdinates(pitchsw, pitch1, pitch2);

      Serial.print("set_swivel: ");
      Serial.print(set_link[0]);
      Serial.print(" set_L1: ");
      Serial.print(set_link[1]);
      Serial.print(" set_L2: ");
      Serial.println(set_link[2]);
    }

    void Stop(void)
    {
      ledcWrite(Swchannel, 0);
      ledcWrite(L1channel, 0);
      ledcWrite(L2channel, 0);
    }

    void drive_rm(void)
    {
      setBNO0551();
      delay(50);
      setBNO0552();
      delay(50);

      set_link[0] = 0;
      set_link[1] = yBNO0551();
      set_link[2] = yBNO0552();
    }

    void get_corrdinates(float sw, float p1, float p2)
    {
      int r = L1 * cos(radians(p1)) + L2 * cos(radians(p2));
      set_Z = L1 * sin(radians(p1)) + L2 * sin(radians(p2));
      set_Y = r * sin(radians(sw));
      set_X = r * cos(radians(sw));
    }

    void calculate_ik()
    {
      //IK--calculate the jointangles
      jointangle0 = atan(set_Y / set_X);
      int R = sqrt(set_Y * set_Y + set_X * set_X);
      jointangle2 = acos((R * R + set_Z * set_Z - L1 * L1 - L2 * L2) / (2 * L1 * L2));
      b = atan((L2 * sin(jointangle2)) / (L2 * cos(jointangle2) + L1));
      a = atan(set_Z / R);
      jointangle1 = a + b;
    }

    void arm_control(int X, int Y, int Z, float deltaT, int Reset)
    {
      if (Reset == 1)
      {
        setBNO0551();
        delay(10);
        setBNO0552();
        delay(10);
      }

      //get current angles
      pitchsw = zBNO0552();
      pitch1 = yBNO0551();
      pitch2 = yBNO0552();

      //      jointangle0 = radians(swivel);
      //      jointangle1 = radians(pitch1);
      //      angle2 = radians(pitch2);
      //      jointangle2 = jointangle1 - angle2;

      //FK--get current coordinates
      get_corrdinates(pitchsw, pitch1, pitch2);

      set_X += X * 0.5;
      set_Y += Y * 0.5;
      set_Z += Z * 0.5;

      float current_pos = set_X * set_X + set_Y * set_Y + set_Z * set_Z;
      if (current_pos <= (max_reach * max_reach))
      {
        if (current_pos >= (min_reach * min_reach))
        {
          calculate_ik();
          Serial.println("1");
        }
        else if (set_Z > 0)
        {
          set_X += 1;
          set_Z += 1;
          //          calculate_ik();
          Serial.print("2");
        }
        else
        {
          set_X += 1;
          set_Z -= 1;
          //          calculate_ik();
          Serial.print("3");
        }
      }
      else if (set_Z >= 0 && set_X >= 0)
      {
        set_X -= 1;
        set_Z -= 1;
        //        calculate_ik();
        Serial.print("4");
      }
      else if (set_Z >= 0 && set_X <= 0)
      {
        set_X += 1;
        set_Z -= 1;
        //        calculate_ik();
        Serial.print("5");
      }
      else
      {
        set_X -= 1;
        set_Z += 1;
        //        calculate_ik();
        Serial.print("6");
      }



      //set the new angles
      set_link[0] = degrees(jointangle0) ;
      set_link[1] = degrees(jointangle1);
      set_link[2] = degrees(jointangle1 - jointangle2);

      Serial.print("set_X: ");
      Serial.print(set_X);
      Serial.print(" | set_Y: ");
      Serial.print(set_Y);
      Serial.print(" | set_Z: ");
      Serial.println(set_Z);

      if (set_link[0] > 135)
      {
        set_link[0] = 135;
      }
      if (set_link[0] < -135)
      {
        set_link[0] = -135;
      }
      if ((pitch1 - pitch2) > 125)
      {
        set_link[2] += 4 ;
      }
      if (set_link[1] > 170)
      {
        set_link[1] = 170;
      }
      if (set_link[1] < -5)
      {
        set_link[1] = -5;
      }

      //      //---BNO055-2-Sw---
      //      float pitchSw = zBNO0552();
      //      delay(10);
      //
      //      //---BNO055-1-L1---
      //      float pitchL1 = yBNO0551();
      //      delay(10);
      //
      //      //---BNO055-2-L2--
      //      float pitchL2 = yBNO0552();
      //      delay(10);

      //      Serial.print("pitSW: ");
      //      Serial.print(pitchsw);
      //      Serial.print(" | pit1: ");
      //      Serial.print(pitch1);
      //      Serial.print(" | pit2: ");
      //      Serial.print(pitch2);
      //      Serial.print("  ||  setSW: ");
      //      Serial.print(set_link[0]);
      //      Serial.print(" | set1: ");
      //      Serial.print(set_link[1]);
      //      Serial.print(" | set2: ");
      //      Serial.println(set_link[2]);

      arm_pid[0].getpid(pitchsw, set_link[0], deltaT, pwmswl, dirswl);
      arm_pid[1].getpid(pitch1, set_link[1], deltaT, pwm1, dir1);
      arm_pid[2].getpid(pitch2, set_link[2], deltaT, pwm2, dir2);

      //Set arm controls
      digitalWrite(dir_swivel, dirswl);
      ledcWrite(Swchannel, pwmswl);
      digitalWrite(dir_link1, dir1);
      ledcWrite(L1channel, pwm1);
      digitalWrite(dir_link2, dir2);
      ledcWrite(L2channel, pwm2);

      //      Serial.print("Swivel:-  Dir:");
      //      Serial.print(dirswl);
      //      Serial.print(",Pwm:");
      //      Serial.print(pwmswl);
      //      Serial.print(" | Link 1:-  Dir:");
      //      Serial.print(dir1);
      //      Serial.print(",Pwm:");
      //      Serial.print(pwm1);
      //      Serial.print(" | Link 2:-  Dir:");
      //      Serial.print(dir2);
      //      Serial.print(",Pwm:");
      //      Serial.println(pwm2);
    }
};
