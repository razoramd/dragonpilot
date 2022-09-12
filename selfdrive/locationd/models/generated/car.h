#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6952590880159753802);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2261899865498132123);
void car_H_mod_fun(double *state, double *out_8676500456739819914);
void car_f_fun(double *state, double dt, double *out_4153484326751475120);
void car_F_fun(double *state, double dt, double *out_74190663820938698);
void car_h_25(double *state, double *unused, double *out_5053605820729364307);
void car_H_25(double *state, double *unused, double *out_1038074172362167343);
void car_h_24(double *state, double *unused, double *out_323218756557611349);
void car_H_24(double *state, double *unused, double *out_5162958507488994083);
void car_h_30(double *state, double *unused, double *out_5629872996159514868);
void car_H_30(double *state, double *unused, double *out_908735225218927273);
void car_h_26(double *state, double *unused, double *out_3610667957368716681);
void car_H_26(double *state, double *unused, double *out_1694928236472479247);
void car_h_27(double *state, double *unused, double *out_7908054221639354854);
void car_H_27(double *state, double *unused, double *out_1266028086581497638);
void car_h_29(double *state, double *unused, double *out_7590963861714711801);
void car_H_29(double *state, double *unused, double *out_1418966569533319457);
void car_h_28(double *state, double *unused, double *out_2864484656338372045);
void car_H_28(double *state, double *unused, double *out_3663432447536211117);
void car_h_31(double *state, double *unused, double *out_2631186891694003320);
void car_H_31(double *state, double *unused, double *out_1068720134239127771);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}