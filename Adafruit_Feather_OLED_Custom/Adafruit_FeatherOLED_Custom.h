/**************************************************************************/
/*!
    @file     Adafruit_FeatherOLED_Custom.h
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

#ifndef _Adafruit_FeatherOLED_Custom_H_
#define _Adafruit_FeatherOLED_Custom_H_

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Adafruit_FeatherOLED_Custom : public Adafruit_SSD1306
{
  protected:
    bool    _batteryIcon;
    bool    _batteryVisible;
    float   _batteryMax;
    float   _batteryMin;

    void drawNumeral(int place, int num);

  public:
    enum
    {
      FOLED_BATTERYICON_NONE      = 0,       // Displays volts
      FOLED_BATTERYICON_THREEBAR  = 1
    };

    // Constructor
    Adafruit_FeatherOLED_Custom ( int reset = -1 ) : Adafruit_SSD1306(reset)
    {
      _batteryIcon        = true;
      _batteryVisible     = true;
      _batteryMax         = 4.2f;
      _batteryMin         = 3.0f;
    }

    void setBattery ( float battery );
    void setConnected ( bool conn );
    void setUnits ( bool imperial );
    void setSpeed ( float mph );
    void setBatteryVisible   ( bool enable )    { _batteryVisible = enable; }
    void setBatteryIcon      ( bool enable )    { _batteryIcon = enable; }
    void setBatteryCuttofs   ( float max, float min) { _batteryMax = max; _batteryMin = min; }

    void init          ( void );
    void clearMsgArea  ( void );
    void refreshIcons  ( void );
};

#endif /* _Adafruit_FeatherOLED_Custom_H_ */
