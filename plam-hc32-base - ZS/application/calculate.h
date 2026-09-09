#ifndef APPLICATION_CALCULATE_H
#define APPLICATION_CALCULATE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#define MY_PI 3.14159265358979323846
extern double och1970_origin_x;
extern double och1970_origin_y;
double delta_calc_x_net(const double x1[12]);
double delta_calc_y_net(const double x1[12]);
void calc_och1970_force(const int16_t mag_raw[12], double *fx, double *fy);

void och1970_set_origin(double x, double y);
void och1970_check_and_update_origin(double cur_x, double cur_y,bool* press_flag, bool* last_status,bool* update_pos_flag);
//bool och1970_press_detect(const uint8_t* frame, int size, int threshold, int min_points);
//double och1970_calc_angle(double fx, double fy, double origin_x, double origin_y);

//double calc_xy(double B[3], bool is_calc_x);
//void calculate_xy_position(double x_T, double y_T, double z_T, double* x_pos, double* y_pos);
#endif // APPLICATION_CALCULATE_H
