/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 */

#include <Wire.h>
#include "PX4Flow.h"

PX4Flow sensor = PX4Flow(); 

long int loop_start;
int loop_time = 20000; // us

void setup()
{
  Wire.begin();       
  Serial.begin(115200);
  
  // start timer
  loop_start = micros();
}

void loop()
{
  static long int dt = 0;
  
  sensor.update();
  
  Serial.print(loop_start); Serial.print(":  ");
  Serial.print(sensor.flow_comp_m_x());Serial.print(", ");
  Serial.print(sensor.flow_comp_m_y());Serial.print(", ");
  Serial.print(sensor.gyro_z_rate()); Serial.print(", ");
  Serial.print(sensor.qual()); Serial.print(", ");
  Serial.println(sensor.ground_distance());

  loop_start = loop_start + loop_time;
  dt = loop_start - micros();
  if (dt > 0) {
    delayMicroseconds(dt);
  }
}

