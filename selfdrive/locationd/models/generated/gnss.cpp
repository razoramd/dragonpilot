#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6112394976586976667) {
   out_6112394976586976667[0] = delta_x[0] + nom_x[0];
   out_6112394976586976667[1] = delta_x[1] + nom_x[1];
   out_6112394976586976667[2] = delta_x[2] + nom_x[2];
   out_6112394976586976667[3] = delta_x[3] + nom_x[3];
   out_6112394976586976667[4] = delta_x[4] + nom_x[4];
   out_6112394976586976667[5] = delta_x[5] + nom_x[5];
   out_6112394976586976667[6] = delta_x[6] + nom_x[6];
   out_6112394976586976667[7] = delta_x[7] + nom_x[7];
   out_6112394976586976667[8] = delta_x[8] + nom_x[8];
   out_6112394976586976667[9] = delta_x[9] + nom_x[9];
   out_6112394976586976667[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1051768255708603758) {
   out_1051768255708603758[0] = -nom_x[0] + true_x[0];
   out_1051768255708603758[1] = -nom_x[1] + true_x[1];
   out_1051768255708603758[2] = -nom_x[2] + true_x[2];
   out_1051768255708603758[3] = -nom_x[3] + true_x[3];
   out_1051768255708603758[4] = -nom_x[4] + true_x[4];
   out_1051768255708603758[5] = -nom_x[5] + true_x[5];
   out_1051768255708603758[6] = -nom_x[6] + true_x[6];
   out_1051768255708603758[7] = -nom_x[7] + true_x[7];
   out_1051768255708603758[8] = -nom_x[8] + true_x[8];
   out_1051768255708603758[9] = -nom_x[9] + true_x[9];
   out_1051768255708603758[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_841499435221476839) {
   out_841499435221476839[0] = 1.0;
   out_841499435221476839[1] = 0;
   out_841499435221476839[2] = 0;
   out_841499435221476839[3] = 0;
   out_841499435221476839[4] = 0;
   out_841499435221476839[5] = 0;
   out_841499435221476839[6] = 0;
   out_841499435221476839[7] = 0;
   out_841499435221476839[8] = 0;
   out_841499435221476839[9] = 0;
   out_841499435221476839[10] = 0;
   out_841499435221476839[11] = 0;
   out_841499435221476839[12] = 1.0;
   out_841499435221476839[13] = 0;
   out_841499435221476839[14] = 0;
   out_841499435221476839[15] = 0;
   out_841499435221476839[16] = 0;
   out_841499435221476839[17] = 0;
   out_841499435221476839[18] = 0;
   out_841499435221476839[19] = 0;
   out_841499435221476839[20] = 0;
   out_841499435221476839[21] = 0;
   out_841499435221476839[22] = 0;
   out_841499435221476839[23] = 0;
   out_841499435221476839[24] = 1.0;
   out_841499435221476839[25] = 0;
   out_841499435221476839[26] = 0;
   out_841499435221476839[27] = 0;
   out_841499435221476839[28] = 0;
   out_841499435221476839[29] = 0;
   out_841499435221476839[30] = 0;
   out_841499435221476839[31] = 0;
   out_841499435221476839[32] = 0;
   out_841499435221476839[33] = 0;
   out_841499435221476839[34] = 0;
   out_841499435221476839[35] = 0;
   out_841499435221476839[36] = 1.0;
   out_841499435221476839[37] = 0;
   out_841499435221476839[38] = 0;
   out_841499435221476839[39] = 0;
   out_841499435221476839[40] = 0;
   out_841499435221476839[41] = 0;
   out_841499435221476839[42] = 0;
   out_841499435221476839[43] = 0;
   out_841499435221476839[44] = 0;
   out_841499435221476839[45] = 0;
   out_841499435221476839[46] = 0;
   out_841499435221476839[47] = 0;
   out_841499435221476839[48] = 1.0;
   out_841499435221476839[49] = 0;
   out_841499435221476839[50] = 0;
   out_841499435221476839[51] = 0;
   out_841499435221476839[52] = 0;
   out_841499435221476839[53] = 0;
   out_841499435221476839[54] = 0;
   out_841499435221476839[55] = 0;
   out_841499435221476839[56] = 0;
   out_841499435221476839[57] = 0;
   out_841499435221476839[58] = 0;
   out_841499435221476839[59] = 0;
   out_841499435221476839[60] = 1.0;
   out_841499435221476839[61] = 0;
   out_841499435221476839[62] = 0;
   out_841499435221476839[63] = 0;
   out_841499435221476839[64] = 0;
   out_841499435221476839[65] = 0;
   out_841499435221476839[66] = 0;
   out_841499435221476839[67] = 0;
   out_841499435221476839[68] = 0;
   out_841499435221476839[69] = 0;
   out_841499435221476839[70] = 0;
   out_841499435221476839[71] = 0;
   out_841499435221476839[72] = 1.0;
   out_841499435221476839[73] = 0;
   out_841499435221476839[74] = 0;
   out_841499435221476839[75] = 0;
   out_841499435221476839[76] = 0;
   out_841499435221476839[77] = 0;
   out_841499435221476839[78] = 0;
   out_841499435221476839[79] = 0;
   out_841499435221476839[80] = 0;
   out_841499435221476839[81] = 0;
   out_841499435221476839[82] = 0;
   out_841499435221476839[83] = 0;
   out_841499435221476839[84] = 1.0;
   out_841499435221476839[85] = 0;
   out_841499435221476839[86] = 0;
   out_841499435221476839[87] = 0;
   out_841499435221476839[88] = 0;
   out_841499435221476839[89] = 0;
   out_841499435221476839[90] = 0;
   out_841499435221476839[91] = 0;
   out_841499435221476839[92] = 0;
   out_841499435221476839[93] = 0;
   out_841499435221476839[94] = 0;
   out_841499435221476839[95] = 0;
   out_841499435221476839[96] = 1.0;
   out_841499435221476839[97] = 0;
   out_841499435221476839[98] = 0;
   out_841499435221476839[99] = 0;
   out_841499435221476839[100] = 0;
   out_841499435221476839[101] = 0;
   out_841499435221476839[102] = 0;
   out_841499435221476839[103] = 0;
   out_841499435221476839[104] = 0;
   out_841499435221476839[105] = 0;
   out_841499435221476839[106] = 0;
   out_841499435221476839[107] = 0;
   out_841499435221476839[108] = 1.0;
   out_841499435221476839[109] = 0;
   out_841499435221476839[110] = 0;
   out_841499435221476839[111] = 0;
   out_841499435221476839[112] = 0;
   out_841499435221476839[113] = 0;
   out_841499435221476839[114] = 0;
   out_841499435221476839[115] = 0;
   out_841499435221476839[116] = 0;
   out_841499435221476839[117] = 0;
   out_841499435221476839[118] = 0;
   out_841499435221476839[119] = 0;
   out_841499435221476839[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_1040724727692524457) {
   out_1040724727692524457[0] = dt*state[3] + state[0];
   out_1040724727692524457[1] = dt*state[4] + state[1];
   out_1040724727692524457[2] = dt*state[5] + state[2];
   out_1040724727692524457[3] = state[3];
   out_1040724727692524457[4] = state[4];
   out_1040724727692524457[5] = state[5];
   out_1040724727692524457[6] = dt*state[7] + state[6];
   out_1040724727692524457[7] = dt*state[8] + state[7];
   out_1040724727692524457[8] = state[8];
   out_1040724727692524457[9] = state[9];
   out_1040724727692524457[10] = state[10];
}
void F_fun(double *state, double dt, double *out_7174788391921015258) {
   out_7174788391921015258[0] = 1;
   out_7174788391921015258[1] = 0;
   out_7174788391921015258[2] = 0;
   out_7174788391921015258[3] = dt;
   out_7174788391921015258[4] = 0;
   out_7174788391921015258[5] = 0;
   out_7174788391921015258[6] = 0;
   out_7174788391921015258[7] = 0;
   out_7174788391921015258[8] = 0;
   out_7174788391921015258[9] = 0;
   out_7174788391921015258[10] = 0;
   out_7174788391921015258[11] = 0;
   out_7174788391921015258[12] = 1;
   out_7174788391921015258[13] = 0;
   out_7174788391921015258[14] = 0;
   out_7174788391921015258[15] = dt;
   out_7174788391921015258[16] = 0;
   out_7174788391921015258[17] = 0;
   out_7174788391921015258[18] = 0;
   out_7174788391921015258[19] = 0;
   out_7174788391921015258[20] = 0;
   out_7174788391921015258[21] = 0;
   out_7174788391921015258[22] = 0;
   out_7174788391921015258[23] = 0;
   out_7174788391921015258[24] = 1;
   out_7174788391921015258[25] = 0;
   out_7174788391921015258[26] = 0;
   out_7174788391921015258[27] = dt;
   out_7174788391921015258[28] = 0;
   out_7174788391921015258[29] = 0;
   out_7174788391921015258[30] = 0;
   out_7174788391921015258[31] = 0;
   out_7174788391921015258[32] = 0;
   out_7174788391921015258[33] = 0;
   out_7174788391921015258[34] = 0;
   out_7174788391921015258[35] = 0;
   out_7174788391921015258[36] = 1;
   out_7174788391921015258[37] = 0;
   out_7174788391921015258[38] = 0;
   out_7174788391921015258[39] = 0;
   out_7174788391921015258[40] = 0;
   out_7174788391921015258[41] = 0;
   out_7174788391921015258[42] = 0;
   out_7174788391921015258[43] = 0;
   out_7174788391921015258[44] = 0;
   out_7174788391921015258[45] = 0;
   out_7174788391921015258[46] = 0;
   out_7174788391921015258[47] = 0;
   out_7174788391921015258[48] = 1;
   out_7174788391921015258[49] = 0;
   out_7174788391921015258[50] = 0;
   out_7174788391921015258[51] = 0;
   out_7174788391921015258[52] = 0;
   out_7174788391921015258[53] = 0;
   out_7174788391921015258[54] = 0;
   out_7174788391921015258[55] = 0;
   out_7174788391921015258[56] = 0;
   out_7174788391921015258[57] = 0;
   out_7174788391921015258[58] = 0;
   out_7174788391921015258[59] = 0;
   out_7174788391921015258[60] = 1;
   out_7174788391921015258[61] = 0;
   out_7174788391921015258[62] = 0;
   out_7174788391921015258[63] = 0;
   out_7174788391921015258[64] = 0;
   out_7174788391921015258[65] = 0;
   out_7174788391921015258[66] = 0;
   out_7174788391921015258[67] = 0;
   out_7174788391921015258[68] = 0;
   out_7174788391921015258[69] = 0;
   out_7174788391921015258[70] = 0;
   out_7174788391921015258[71] = 0;
   out_7174788391921015258[72] = 1;
   out_7174788391921015258[73] = dt;
   out_7174788391921015258[74] = 0;
   out_7174788391921015258[75] = 0;
   out_7174788391921015258[76] = 0;
   out_7174788391921015258[77] = 0;
   out_7174788391921015258[78] = 0;
   out_7174788391921015258[79] = 0;
   out_7174788391921015258[80] = 0;
   out_7174788391921015258[81] = 0;
   out_7174788391921015258[82] = 0;
   out_7174788391921015258[83] = 0;
   out_7174788391921015258[84] = 1;
   out_7174788391921015258[85] = dt;
   out_7174788391921015258[86] = 0;
   out_7174788391921015258[87] = 0;
   out_7174788391921015258[88] = 0;
   out_7174788391921015258[89] = 0;
   out_7174788391921015258[90] = 0;
   out_7174788391921015258[91] = 0;
   out_7174788391921015258[92] = 0;
   out_7174788391921015258[93] = 0;
   out_7174788391921015258[94] = 0;
   out_7174788391921015258[95] = 0;
   out_7174788391921015258[96] = 1;
   out_7174788391921015258[97] = 0;
   out_7174788391921015258[98] = 0;
   out_7174788391921015258[99] = 0;
   out_7174788391921015258[100] = 0;
   out_7174788391921015258[101] = 0;
   out_7174788391921015258[102] = 0;
   out_7174788391921015258[103] = 0;
   out_7174788391921015258[104] = 0;
   out_7174788391921015258[105] = 0;
   out_7174788391921015258[106] = 0;
   out_7174788391921015258[107] = 0;
   out_7174788391921015258[108] = 1;
   out_7174788391921015258[109] = 0;
   out_7174788391921015258[110] = 0;
   out_7174788391921015258[111] = 0;
   out_7174788391921015258[112] = 0;
   out_7174788391921015258[113] = 0;
   out_7174788391921015258[114] = 0;
   out_7174788391921015258[115] = 0;
   out_7174788391921015258[116] = 0;
   out_7174788391921015258[117] = 0;
   out_7174788391921015258[118] = 0;
   out_7174788391921015258[119] = 0;
   out_7174788391921015258[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_1515563691164347021) {
   out_1515563691164347021[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2025915146887303674) {
   out_2025915146887303674[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2025915146887303674[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2025915146887303674[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2025915146887303674[3] = 0;
   out_2025915146887303674[4] = 0;
   out_2025915146887303674[5] = 0;
   out_2025915146887303674[6] = 1;
   out_2025915146887303674[7] = 0;
   out_2025915146887303674[8] = 0;
   out_2025915146887303674[9] = 0;
   out_2025915146887303674[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_9172804088573809389) {
   out_9172804088573809389[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_7512139862993975981) {
   out_7512139862993975981[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7512139862993975981[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7512139862993975981[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7512139862993975981[3] = 0;
   out_7512139862993975981[4] = 0;
   out_7512139862993975981[5] = 0;
   out_7512139862993975981[6] = 1;
   out_7512139862993975981[7] = 0;
   out_7512139862993975981[8] = 0;
   out_7512139862993975981[9] = 1;
   out_7512139862993975981[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_7817427610718480288) {
   out_7817427610718480288[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3276206153451324772) {
   out_3276206153451324772[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[6] = 0;
   out_3276206153451324772[7] = 1;
   out_3276206153451324772[8] = 0;
   out_3276206153451324772[9] = 0;
   out_3276206153451324772[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_7817427610718480288) {
   out_7817427610718480288[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3276206153451324772) {
   out_3276206153451324772[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3276206153451324772[6] = 0;
   out_3276206153451324772[7] = 1;
   out_3276206153451324772[8] = 0;
   out_3276206153451324772[9] = 0;
   out_3276206153451324772[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6112394976586976667) {
  err_fun(nom_x, delta_x, out_6112394976586976667);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_1051768255708603758) {
  inv_err_fun(nom_x, true_x, out_1051768255708603758);
}
void gnss_H_mod_fun(double *state, double *out_841499435221476839) {
  H_mod_fun(state, out_841499435221476839);
}
void gnss_f_fun(double *state, double dt, double *out_1040724727692524457) {
  f_fun(state,  dt, out_1040724727692524457);
}
void gnss_F_fun(double *state, double dt, double *out_7174788391921015258) {
  F_fun(state,  dt, out_7174788391921015258);
}
void gnss_h_6(double *state, double *sat_pos, double *out_1515563691164347021) {
  h_6(state, sat_pos, out_1515563691164347021);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2025915146887303674) {
  H_6(state, sat_pos, out_2025915146887303674);
}
void gnss_h_20(double *state, double *sat_pos, double *out_9172804088573809389) {
  h_20(state, sat_pos, out_9172804088573809389);
}
void gnss_H_20(double *state, double *sat_pos, double *out_7512139862993975981) {
  H_20(state, sat_pos, out_7512139862993975981);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7817427610718480288) {
  h_7(state, sat_pos_vel, out_7817427610718480288);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3276206153451324772) {
  H_7(state, sat_pos_vel, out_3276206153451324772);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7817427610718480288) {
  h_21(state, sat_pos_vel, out_7817427610718480288);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3276206153451324772) {
  H_21(state, sat_pos_vel, out_3276206153451324772);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
