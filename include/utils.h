#pragma once

#include "Arduino.h"

inline float rand_range(float min, float max)
{
  float unit = ((float)random()) / RAND_MAX;
  return (max - min) * unit + min;
}