// SPDX-FileCopyrightText: 2019 teejaydub for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#if 1  // Change to 1 to enable this code (must enable ONE user*.cpp only!)
// CORRESPONDING LINE IN HeatSensor.cpp MUST ALSO BE ENABLED!

#include "globals.h"
#include <Wire.h>
#include "person_sensor.h"

// How long to wait between reading the sensor. The sensor can be read as
// frequently as you like, but the results only change at about 5FPS, so
// waiting for 200ms is reasonable.
const int32_t SAMPLE_DELAY_MS = 200;

uint32_t timeout = 0L;
uint32_t noFaceTimeout = 0L;

float stepX(0.01), stepY(0.01);
float eX(0.0), eY(0.0), eXM1(0.0), eYM1(0.0), eTXM1(0.0), eTYM1(0.0);
float fa(1.0); // 0.0-> full filter, 1.0-> no filter
uint8_t debug(0x00);

person_sensor_results_t results;
typedef struct
{
   uint8_t x0;
   uint8_t y0;
   uint8_t x1;
   uint8_t y1;
   uint8_t w;
   uint8_t h;
   uint8_t cx;
   uint8_t cy;
   const person_sensor_face_t *face;
} box_t;

box_t closest;

void TrackFace(uint32_t m);

char *NextToken(char *buf, char *sep)
{
   return(strtok(buf, sep));
}

void SetEyeOffset(char *buf)
{
   float x0(-100), x1(-100), y0(-100), y1(-100);

   char *tok(NextToken(buf, ","));
   if(tok != NULL)
   {
     x0 = atof(tok);
   }
   tok = NextToken(NULL, ",");
   if(tok != NULL)
   {
     y0 = atof(tok);
   }
   tok = NextToken(NULL, ",");
   if(tok != NULL)
   {
     x1 = atof(tok);
   }
   tok = NextToken(NULL, ",");
   if(tok != NULL)
   {
     y1 = atof(tok);
   }
   if(x0 >= -1.0 && x0 <= 1.0 &&
      y0 >= -1.0 && y0 <= 1.0 &&
      x1 >= -1.0 && x1 <= 1.0 &&
      y1 >= -1.0 && y1 <= 1.0)
   {
      eye[0].eyeOffsetX = x0;
      eye[0].eyeOffsetY = y0;
      eye[1].eyeOffsetX = x1;
      eye[1].eyeOffsetY = y1;
      Serial.print(F("Offset: "));
   }
   else
   {
      Serial.print(F("Invalid Offset Inputs"));
   }
   Serial.print(eye[0].eyeOffsetX);Serial.print(", ");Serial.print(eye[0].eyeOffsetY);Serial.print(", ");
   Serial.print(eye[1].eyeOffsetX);Serial.print(", ");Serial.println(eye[1].eyeOffsetY);
}

void SetEyePosition(char *buf)
{
   float x(-100), y(-100);

   char *tok(NextToken(buf, ","));
   if(tok != NULL)
   {
     x = atof(tok);
   }
   tok = NextToken(NULL, ",");
   if(tok != NULL)
   {
     y = atof(tok);
   }
   
   if(x >= -1.0 && x <= 1.0 &&
      y >= -1.0 && y <= 1.0)
   {
      eyeTargetX = x;
      eyeTargetY = y;
      Serial.print(F("Position: "));
      Serial.print(eyeTargetX);Serial.print(F(", "));Serial.println(eyeTargetY);
   }
   else
   {
      Serial.print(F("Invalid Offset Inputs"));
   }
}

uint8_t ToggleDebug()
{
   // Wire.beginTransmission(PERSON_SENSOR_I2C_ADDRESS);
   // Wire.write(PERSON_SENSOR_REG_DEBUG_MODE);
   // Wire.requestFrom(PERSON_SENSOR_I2C_ADDRESS, 1);
   // uint8_t debug = Wire.read();
   // Wire.endTransmission();

   Serial.print(F("Debug: "));Serial.print(debug);
   debug = debug == 0x01 ? 0x00 : 0x01;
   person_sensor_write_reg(PERSON_SENSOR_REG_DEBUG_MODE, debug);
   
   // Wire.beginTransmission(PERSON_SENSOR_I2C_ADDRESS);
   // Wire.write(PERSON_SENSOR_REG_DEBUG_MODE);
   // Wire.requestFrom(PERSON_SENSOR_I2C_ADDRESS, 1);
   // debug = Wire.read();
   // Wire.endTransmission();
   Serial.print(F(", "));Serial.println(debug);

   return(debug);
}

#define IN_BUFF_LEN 128
char IN_BUFFER[IN_BUFF_LEN] = { '\0' };
void ProcessMessage(char *buf)
{
   switch(buf[0])
   {
     case('O'):
        SetEyeOffset(&buf[1]);
        break;
      case('P'):
         SetEyePosition(&buf[1]);
         break;
      case('D'):
         ToggleDebug();
      default:
         break;
   }
}

void ReadMessage()
{
   char c('\0');
   uint8_t i(0);
   while(Serial.available())
   {
      c = Serial.read();
      if(c == '\n' ||
         i >= IN_BUFF_LEN)
      {
         Serial.print(F("MSG: "));Serial.println(IN_BUFFER);
         ProcessMessage(IN_BUFFER);
         memset(IN_BUFFER, '\0', IN_BUFF_LEN);
         break;
      }
      IN_BUFFER[i] = c;
      i++;
   }
}
// This file provides a crude way to "drop in" user code to the eyes,
// allowing concurrent operations without having to maintain a bunch of
// special derivatives of the eye code (which is still undergoing a lot
// of development). Just replace the source code contents of THIS TAB ONLY,
// compile and upload to board. Shouldn't need to modify other eye code.

// User globals can go here, recommend declaring as static, e.g.:
// static int foo = 42;

// Called once near the end of the setup() function. If your code requires
// a lot of time to initialize, make periodic calls to yield() to keep the
// USB mass storage filesystem alive.
void user_setup(void) {
  showSplashScreen = false;
  moveEyesRandomly = false;
  eyeTargetX = 0.0;
  eyeTargetY = 0.0;

  // Alignment offset for Sadako Yamamura
  eye[0].eyeOffsetX = 0.25;
  eye[0].eyeOffsetY = -0.1;
#if (NUM_EYES > 1)
  eye[1].eyeOffsetX = -0.20;
  eye[1].eyeOffsetY = -0.1;
#endif
   limitsX[0] = 0.5;   // R
   limitsX[1] = -0.75; // L
   limitsY[0] = -0.35; // U
   limitsY[1] = 0.25;  // D

  Wire.begin();

  person_sensor_write_reg(0x07, debug);
}

// Called periodically during eye animation. This is invoked in the
// interval before starting drawing on the last eye (left eye on MONSTER
// M4SK, sole eye on HalloWing M0) so it won't exacerbate visible tearing
// in eye rendering. This is also SPI "quiet time" on the MONSTER M4SK so
// it's OK to do I2C or other communication across the bridge.
void user_loop(void)
{
   ReadMessage();
   uint32_t m(millis());
   TrackFace(m);
}

void TrackFace(uint32_t m)
{
  if((m - timeout) >= SAMPLE_DELAY_MS)
  {
    timeout = millis();

    if (!person_sensor_read(&results))
    {
       return;
    }
    else
    {
      if(results.num_faces == 0)
      {
         if((m - noFaceTimeout) >= 3000)
         {
            if(moveEyesRandomly == false)
            {
               Serial.println("Random on");
               moveEyesRandomly = true;
            }
         }
      }
      else
      {
         if(moveEyesRandomly == true)
         {
           Serial.println("Random off");
           moveEyesRandomly = false;
         }
         noFaceTimeout = m;
        
        closest.face = nullptr;
        closest.w = 0;
        closest.h = 0;
        uint8_t x0, x1;
        for (int i = 0; i < results.num_faces; ++i) 
        {
          box_t next;
          next.face = &results.faces[i];
          x1 = 256 - next.face->box_left;
          x0 = 256 - next.face->box_right;
          next.w = x1 - x0;
          next.h = next.face->box_bottom - next.face->box_top;

          if(next.w > closest.w && next.h > closest.h)
          {
            closest.face = &results.faces[i];
            closest.w = next.w;
            closest.h = next.h;
            closest.cx = x0 + (closest.w * 0.5);
            closest.cy = closest.face->box_top + (closest.h * 0.5);
          

            // const person_sensor_face_t* face = closest.face;//&results.faces[i];
            // Serial.print("Face #");
            // Serial.print(i);
            // Serial.print(": ");
            // Serial.print(face->box_confidence);
            // Serial.print(" confidence, (");
            // Serial.print(x0);//face->box_left);
            // Serial.print(", ");
            // Serial.print(face->box_top);
            // Serial.print("), (");
            // Serial.print(x1);//face->box_right);
            // Serial.print(", ");
            // Serial.print(face->box_bottom);
            // Serial.print("), ");
            // if (face->is_facing)
            // {
            //   Serial.println("facing");
            // }
            // else
            // {
            //   Serial.println("not facing");
            // }
          }
        }
      }
      if(closest.face != nullptr)
      {
          eXM1 = eX;
          eYM1 = eY;
          // Set values for the new X and Y.
          eX = -((closest.cx / 256.0) - 0.5);
          eY = (closest.cy / 256.0) - 0.5;
          eye[0].eyeOffsetX = eX;
          if(debug == true)
          {
             //Serial.print("ex,ey: ");Serial.print(eX);Serial.print(",");Serial.println(eY);
             Serial.print(F("ex,ey: "));Serial.print(eyeTargetX);Serial.print(F(","));Serial.println(eyeTargetY);
          }
      }
    }

    // if(fabs(eyeTargetX) >= 1.0)
    // {
    //   stepX *= -1;
    // }
    // eyeTargetX += stepX;
    // Serial.println(eyeTargetX);
   
    // char val[10] = {'\0'};
    // uint8_t i(0);
    // while(Serial.available())
    // {
    //   char c = Serial.read();
    //   if(c == '\n')
    //   {
    //      fa = atof(val);
    //      break;
    //   }
    //   val[i++] = c;
    // }
    //Serial.println(fa);
  }

  else
  {
     eyeTargetX = eX;//((1.0 - fa) * eTXM1) + (fa * ((eX + eXM1) * 0.5));
     eyeTargetY = eY;//((1.0 - fa) * eTYM1) + (fa * ((eY + eYM1) * 0.5));
     eTXM1 = eyeTargetX;
     eTYM1 = eyeTargetY;
  }
}

#endif  // 0
