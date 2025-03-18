#pragma once

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

double correctBearing(double bearing);
int sgn(double val);
double bound(double value, double low, double high);

#endif