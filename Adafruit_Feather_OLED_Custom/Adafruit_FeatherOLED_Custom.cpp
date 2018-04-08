/**************************************************************************/
/*!
    @file     Adafruit_FeatherOLED_Custom.cpp
    @author   ffboard1029

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018 Vesc-Speedometer
    based on code by Adafruit Industries (adafruit.com) Copyright (c) 2016, Adafruit_FeatherOLED
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include "Adafruit_FeatherOLED_Custom.h"

/******************************************************************************/
/*!
    @brief  Custom battery function modified from Adafruit_FeatherOLED
*/
/******************************************************************************/
void Adafruit_FeatherOLED_Custom::setBattery ( float battery )
{
    #define BATTTEXT_STARTX     77
    #define BATTTEXT_STARTY     0
    #define BATTICON_STARTX     110
    #define BATTICON_STARTY     0
    #define BATTICON_WIDTH      18
    #define BATTICON_BARWIDTH4  ((BATTICON_WIDTH - 6) / 4)

    #define UNITS_STARTX        77
    #define UNITS_STARTY        18


  if (_batteryVisible)
  {
    fillRect(BATTTEXT_STARTX, BATTICON_STARTY, 128, 8, BLACK);
    // Render the voltage in text
    setTextSize(1);
    setCursor(BATTTEXT_STARTX, BATTTEXT_STARTY);
    print(battery, 2);
    println("V");

    // Render the battery icon if requested
    if (_batteryIcon)
    {
      float voltPerLvl = (_batteryMax - _batteryMin) / 5.0f;

      // Draw the base of the battery
      drawLine( BATTICON_STARTX + 1,
                BATTICON_STARTY,
                BATTICON_STARTX + BATTICON_WIDTH - 4,
                BATTICON_STARTY,
                WHITE);
      drawLine( BATTICON_STARTX,
                BATTICON_STARTY + 1,
                BATTICON_STARTX,
                BATTICON_STARTY + 5,
                WHITE);
      drawLine( BATTICON_STARTX + 1,
                BATTICON_STARTY + 6,
                BATTICON_STARTX + BATTICON_WIDTH - 4,
                BATTICON_STARTY + 6,
                WHITE);
      drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 3,
                BATTICON_STARTY + 1,
                WHITE);
      drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 2,
                BATTICON_STARTY + 1,
                WHITE);
      drawLine( BATTICON_STARTX + BATTICON_WIDTH - 1,
                BATTICON_STARTY + 2,
                BATTICON_STARTX + BATTICON_WIDTH - 1,
                BATTICON_STARTY + 4,
                WHITE);
      drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 2,
                BATTICON_STARTY + 5,
                WHITE);
      drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 3,
                BATTICON_STARTY + 5,
                WHITE);
      drawPixel(BATTICON_STARTX + BATTICON_WIDTH - 3,
                BATTICON_STARTY + 6,
                WHITE);

      // Draw the appropriate number of bars
      if (battery > _batteryMax )
      {
        // USB (Solid Rectangle)
        fillRect(BATTICON_STARTX + 2,     // X
                 BATTICON_STARTY + 2,     // Y
                 BATTICON_BARWIDTH4 * 4,  // W
                 3,                       // H
                 WHITE);
      }
      else if ((battery <= _batteryMax) && (battery >= _batteryMax - voltPerLvl))
      {
        // Four bars
        for (uint8_t i = 0; i < 4; i++)
        {
          fillRect(BATTICON_STARTX + 2 + (i * BATTICON_BARWIDTH4),
                   BATTICON_STARTY + 2,
                   BATTICON_BARWIDTH4 - 1,
                   3,
                   WHITE);
        }
      }
      else if ((battery < _batteryMax - voltPerLvl) && (battery >= _batteryMax - voltPerLvl*2))
      {
        // Three bars
        for (uint8_t i = 0; i < 3; i++)
        {
          fillRect(BATTICON_STARTX + 2 + (i * BATTICON_BARWIDTH4),
                   BATTICON_STARTY + 2,
                   BATTICON_BARWIDTH4 - 1,
                   3,
                   WHITE);
        }
      }
      else if ((battery < _batteryMax - voltPerLvl*2) && (battery >= _batteryMax - voltPerLvl*3))
      {
          // Two bars
          for (uint8_t i = 0; i < 2; i++)
          {
            fillRect(BATTICON_STARTX + 2 + (i * BATTICON_BARWIDTH4),
                     BATTICON_STARTY + 2,
                     BATTICON_BARWIDTH4 - 1,
                     3,
                     WHITE);
          }
      }
      else if ((battery < _batteryMax - voltPerLvl*3) && (battery >= _batteryMax - voltPerLvl*4))
      {
          // One bar
          fillRect(BATTICON_STARTX + 2,
                   BATTICON_STARTY + 2,
                   BATTICON_BARWIDTH4 - 1,
                   3,
                   WHITE);
      }
      else if ((battery < _batteryMax - voltPerLvl*4) && (battery >= _batteryMin + .05f))
      {
        // No bars
      }
      else
      {
          //slash through icon, TURN OFF NOW
          drawLine(BATTICON_STARTX - 1,
                   BATTICON_STARTY + 7,
                   BATTICON_STARTX + BATTICON_WIDTH,
                   BATTICON_STARTY,
                   WHITE);
      }
    }
  }
}

/******************************************************************************/
/*!
    @brief  Renders the bluetooth connection status icon
*/
/******************************************************************************/
void Adafruit_FeatherOLED_Custom::setConnected ( bool conn )
{
    if(conn)
    {
        drawLine(124, 19, 124, 31, WHITE);
        drawLine(124, 19, 127, 22, WHITE);
        drawLine(127, 22, 121, 28, WHITE);
        drawLine(121, 22, 127, 28, WHITE);
        drawLine(127, 28, 124, 31, WHITE);
    }
    else
    {
        fillRect(121, 19, 128, 31, BLACK);
    }
}

/******************************************************************************/
/*!
    @brief  Initialises the display (always call this first!)
*/
/******************************************************************************/
void Adafruit_FeatherOLED_Custom::init ( void )
{
  // Generate the high voltage from the 3.3v line internally and
  // initialize with the I2C addr 0x3C (for the 128x32)
  begin(SSD1306_SWITCHCAPVCC, 0x3C);
  setTextSize(1);
  setTextColor(WHITE);
  clearDisplay();
}

/******************************************************************************/
/*!
    @brief  Dislpays MPH or kmh
*/
/******************************************************************************/
void Adafruit_FeatherOLED_Custom::setUnits ( bool imperial )
{
    fillRect(UNITS_STARTX, UNITS_STARTY, UNITS_STARTX + 40, 16, BLACK);
    setTextSize(2);
    setCursor(UNITS_STARTX, UNITS_STARTY);
    if(imperial)
    {
        print("MPH");
    }
    else
    {
        print("kmh");
    }
}

/******************************************************************************/
/*!
    @brief  draws the speed, in mph or kph
*/
/******************************************************************************/
void Adafruit_FeatherOLED_Custom::setSpeed ( float speed )
{
    fillRect(0, 0, 70, 32, BLACK);

    int tens = ((int)speed) / 10;

    if(tens > 0)
    {
      drawNumeral(2, tens);
    }
    int ones = (int)speed % 10;

    drawNumeral(1, ones);
    int tenths = (int)(speed * 10) % 10;
    drawNumeral(0, 0);
    drawNumeral(-1, tenths);
}

/******************************************************************************/
/*!
    @brief  Draws a numeral, full height, place moves the number left or right
*/
/******************************************************************************/
void Adafruit_FeatherOLED_Custom::drawNumeral(int place, int num)
{
    if(num > 9 || num < 0 || place > 2 || place < -1)
    {
        return;
    }
    int startx = 0;
    int starty = 0;
    int height = 32;
    int width = 15;

    if(place == 0)
    {
        fillRect(48, 28, 4, 4, WHITE);
        return;
    }
    else if (place == -1)
    {
        startx = 55;
    }
    else if (place == 1)
    {
        startx = 29;
    }
    else if (place == 2)
    {
        startx = 9;
    }

    if(num == 0)
    {
        fillRect(startx, starty + 4, 2, height - 8, WHITE);
        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        fillRect(startx + 4, height - 2, width - 8, 2, WHITE);

        fillRect(startx + 2, starty + 2, 2, 2, WHITE);
        fillRect(startx + 2, height - 4, 2, 2, WHITE);
        startx += width;
        fillRect(startx - 2, starty + 4, 2, height - 8, WHITE);
        fillRect(startx - 4, starty + 2, 2, 2, WHITE);
        fillRect(startx - 4, height - 4, 2, 2, WHITE);
    }
    else if(num == 1)
    {
        fillRect(startx + 4, height - 2, width - 5, 2, WHITE);
        startx += width / 2 + 1;
        fillRect(startx, starty, 2, height, WHITE);
        fillRect(startx - 2, starty + 3, 2, 2, WHITE);
    }
    else if(num == 2)
    {
        /*   alternate 2
        starty += 4;
        fillRect(startx, starty, 2, 4, WHITE);
        starty -= 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        starty -= 2;
        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        starty += 2;
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx + width - 2, starty, 2, 10, WHITE);
        starty += 10;
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        starty += 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx, starty, 2, height - starty, WHITE);
        fillRect(startx, height - 2, width, 2, WHITE);
        */
        starty += 4;
        fillRect(startx, starty, 2, 4, WHITE);
        starty -= 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        starty -= 2;
        fillRect(startx + 4, starty, width - 10, 2, WHITE);
        starty += 2;
        fillRect(startx + width - 6, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx + width - 4, starty, 2, 8, WHITE);
        starty += 8;
        for(int i = 0; i < 10; i++)
        {
            fillRect(startx + width - 5 - 1 * i, starty, 2, 2, WHITE);
            starty += 2;
        }
        fillRect(startx + 1, height - 2, width - 2, 2, WHITE);
    }
    else if(num == 3)
    {
        fillRect(startx + 2, starty, width - 2, 2, WHITE);
        for(int i = 0; i < 8; i++)
        {
            fillRect(startx + width - 1 - 1 * i, starty, 2, 2, WHITE);
            starty += 2;
        }
        fillRect(startx + width - 8, starty - 2, 4, 2, WHITE);
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        //bottom of loop
        fillRect(startx + width - 2, starty, 2, height - 4 - starty, WHITE);
        starty = height - 4;
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        starty -= 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        starty -= 4;
        fillRect(startx, starty, 2, 4, WHITE);

    }
    else if(num == 4)
    {
        starty = 10*2;
        fillRect(startx, starty, width, 2, WHITE);
        for(int i = 0; i < 10; i++)
        {
            fillRect(startx, starty - 2 * i, 2, 2, WHITE);
            startx += 1;
        }
        fillRect(startx, 0, 2, height, WHITE);
    }
    else if(num == 5)
    {
        fillRect(startx, starty, width, 2, WHITE);
        fillRect(startx, starty, 2, 11, WHITE);
        starty += 11;
        fillRect(startx, starty, width - 4, 2, WHITE);
        starty += 2;
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        //bottom of loop
        fillRect(startx + width - 2, starty, 2, height - 4 - starty, WHITE);
        starty = height - 4;
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        starty -= 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        starty -= 4;
        fillRect(startx, starty, 2, 4, WHITE);

    }
    else if(num == 6)
    {
        fillRect(startx + 6, starty, 7, 2, WHITE);
        starty += 4;
        for(int i = 0; i < 4; i++)
        {
            fillRect(startx + 1 + i * 1, starty - i * 1, 2, 2, WHITE);
        }
        starty += 2;
        fillRect(startx, starty, 2, height - 10, WHITE);
        starty += 9;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        starty -= 2;
        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        starty += 2;
        fillRect(startx + width - 4, starty, 2, 2, WHITE);

        starty += 2;
        fillRect(startx + width - 2, starty, 2, height - 4 - starty, WHITE);
        starty = height - 4;
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        starty -= 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
    }
    else if(num == 7)
    {
        fillRect(startx, starty, width, 2, WHITE);

        starty = height;
        for(int i = 0; i < 14; i++)
        {
            fillRect(startx, starty, 2, 2, WHITE);
            startx += 1; starty -= 2;
        }
        fillRect(startx - 1, 2, 2, starty, WHITE);
    }
    else if(num == 8)
    {
        starty += 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        fillRect(startx + 4, starty - 2, width - 8, 2, WHITE);
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx, starty, 2, starty + 5, WHITE);
        fillRect(startx + width - 2, starty, 2, starty + 5, WHITE);
        starty += 9;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;

        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        starty += 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        fillRect(startx + width - 4, starty, 2, 2, WHITE);

        starty += 2;
        fillRect(startx + width - 2, starty, 2, height - 4 - starty, WHITE);
        fillRect(startx, starty, 2, height - 4 - starty, WHITE);
        starty = height - 4;
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        starty -= 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
    }
    else if(num == 9)
    {
        starty += 2;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        fillRect(startx + 4, starty - 2, width - 8, 2, WHITE);
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx, starty, 2, starty + 7, WHITE);
        fillRect(startx + width - 2, starty, 2, starty + 18, WHITE);
        starty += 11;
        fillRect(startx + 2, starty, 2, 2, WHITE);
        fillRect(startx + width - 4, starty, 2, 2, WHITE);
        starty += 2;
        fillRect(startx + 4, starty, width - 8, 2, WHITE);
        starty = height - 3;
        for(int i = 0; i < 4; i++)
        {
            fillRect(startx + 9 + i * 1, starty - i * 1, 2, 2, WHITE);
        }
        starty += 1;
        fillRect(startx + 3, height - 2, 6, 2, WHITE);
    }
}
