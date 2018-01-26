#ifndef GET_ROBOT_XFORM
#define GET_ROBOT_XFORM

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "get_robot_xform_types.h"
#include "globals.h"

// Function Declarations
static void Pbt_disk(double theta_tL, double delta_t, const double length[], double segmentNum, double
                     diskNum, double P_bt_disk[3]);
static void Pbt_tl(double theta_tL, double delta_t, double Lt, double P_bt_tl[3]);
static void Rbi_disk(double theta_tL, double delta_t, const double length[], double segmentNum, double
                     diskNum, double R_bi_disk[9]);
//static void directKin(const double psi[7], double frame_i, double frame_j, const
//                      double L[3], double homTranij[16]);
void directKin(const double psi[7], double frame_i, double frame_j, const
double L[3], double homTranij[16]);
static void rot2Quat(const double R[9], double q[4]);


void get_Robot_Xform(const double psi[7], const double length[3], double xFormVec[91]);


#endif // GET_ROBOT_XFORM

