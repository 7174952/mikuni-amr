#ifndef UTILS_H
#define UTILS_H
#include <stdint.h>

double rad_diff(double past, double current); // Compares 2 radian values and returns difference between them.

uint8_t convert_mps_to_whill_speed(float);
uint8_t convert_mpss_to_whill_acc(float);

uint8_t convert_radps_to_whill_speed(float tread,float radps);
uint8_t convert_radpss_to_whill_acc(float tread, float radpss);

#endif // UTILS_H
