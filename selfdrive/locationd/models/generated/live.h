#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1004438546149370339);
void live_err_fun(double *nom_x, double *delta_x, double *out_8969152691296427567);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3659399576229617090);
void live_H_mod_fun(double *state, double *out_2949472263592139464);
void live_f_fun(double *state, double dt, double *out_1878871363542729359);
void live_F_fun(double *state, double dt, double *out_6484486210560443063);
void live_h_4(double *state, double *unused, double *out_8951149540069868237);
void live_H_4(double *state, double *unused, double *out_7341580059135160500);
void live_h_9(double *state, double *unused, double *out_2591271411914187821);
void live_H_9(double *state, double *unused, double *out_3817945079309943646);
void live_h_10(double *state, double *unused, double *out_7505501518899847905);
void live_H_10(double *state, double *unused, double *out_1027820157764279124);
void live_h_12(double *state, double *unused, double *out_2509465980707663534);
void live_H_12(double *state, double *unused, double *out_960321682092427504);
void live_h_35(double *state, double *unused, double *out_5209909461446594374);
void live_H_35(double *state, double *unused, double *out_692472668566926915);
void live_h_32(double *state, double *unused, double *out_2608196464487643323);
void live_H_32(double *state, double *unused, double *out_6489542878835858806);
void live_h_13(double *state, double *unused, double *out_6693706027089483150);
void live_H_13(double *state, double *unused, double *out_4381346917234443660);
void live_h_14(double *state, double *unused, double *out_2591271411914187821);
void live_H_14(double *state, double *unused, double *out_3817945079309943646);
void live_h_33(double *state, double *unused, double *out_747017267034963486);
void live_H_33(double *state, double *unused, double *out_2458084336071930689);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}