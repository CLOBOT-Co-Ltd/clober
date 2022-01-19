/*
 * Copyright 2022 CLOBOT Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <clober_serial/clober_utils.hpp>


float CloberUtils::toRPM(float w)
{
  float rpm = (w * 60.0) / (2 * PI);
  return rpm;
}

float CloberUtils::toVelocity(float rpm)
{
  float w = (rpm * 2.0 * PI) / 60.0;
  return w;
}

float CloberUtils::toRad(float enc, int ppr)
{
  float rad = (enc * 2.0 * PI) / ppr;
  return rad;
}
