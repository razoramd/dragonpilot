#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6112394976586976667);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_1051768255708603758);
void gnss_H_mod_fun(double *state, double *out_841499435221476839);
void gnss_f_fun(double *state, double dt, double *out_1040724727692524457);
void gnss_F_fun(double *state, double dt, double *out_7174788391921015258);
void gnss_h_6(double *state, double *sat_pos, double *out_1515563691164347021);
void gnss_H_6(double *state, double *sat_pos, double *out_2025915146887303674);
void gnss_h_20(double *state, double *sat_pos, double *out_9172804088573809389);
void gnss_H_20(double *state, double *sat_pos, double *out_7512139862993975981);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7817427610718480288);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3276206153451324772);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7817427610718480288);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3276206153451324772);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}