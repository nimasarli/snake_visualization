//  This function calculates direct kinematic homogenous transformation of
//  all the frames of TURBT robot and stack them up in xFormVec. It requires
//  direktKin function to calculate the homogenous xform of the end of each
//  segment
//
//  Author: Nima
//
//  Created       3/5/2016
//
//  Last edited   3/20/2016
//

//
// parse constants
// Arguments    : const double psi[7]
//                double xFormVec[91]
// Return Type  : void
//
//

// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: get_robot_xform.cpp
//
// MATLAB Coder version            : 2.8
// C/C++ source code generated on  : 20-Mar-2016 22:01:21
//

#include "get_robot_xform.h"
void get_Robot_Xform(const double psi[7], const double length[3], double xFormVec[91])
{
    double theta1l;
    double delta1;
    double theta2l;
    double delta2;
    double theta3l;
    double delta3;
    double qIns; //added by Nima manually 6/18/2016
    double dv0[4];
    static const double dv1[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

    int i;
    double R_bi_disk[9];
    double dv2[3];
    int b_i;
    double homXformSeg1End[16];
//    static const double dv3[3] = { 0.032, 0.024, 0.0155 };
    static double dv3[3]; //modified by Nima 11/25/2017
    dv3[0] = length[0];
    dv3[1] = length[1];
    dv3[2] = length[2];

    double homXform[16];
    double b_R_bi_disk[16];
    int i0;
    static const signed char iv0[4] = { 0, 0, 0, 1 };

    int i1;
    double b_homXform[9];
    double dv4[7];

    //  parse input
    theta1l = psi[0];
    delta1 = psi[1];
    theta2l = psi[2];
    delta2 = psi[3];
    theta3l = psi[4];
    delta3 = psi[5];
    qIns = psi[6]; //added by Nima manually 6/18/2016

    //  preallocation
    memset(&xFormVec[0], 0, 91U * sizeof(double));

    //  translation
    rot2Quat(dv1, dv0);
    for (i = 0; i < 4; i++) {
        xFormVec[i] = dv0[i];
    }

    xFormVec[4] = 0.0;
    xFormVec[5] = 0.0;
    xFormVec[6] = psi[6];

    //  1st segment
    for (i = 0; i < 5; i++) {
        Rbi_disk(theta1l, delta1, length, 1.0, 1.0 + (double)i, R_bi_disk);
        rot2Quat(R_bi_disk, dv0);
        Pbt_disk(theta1l, delta1, length, 1.0, 1.0 + (double)i, dv2);
        dv2[2] = dv2[2] + qIns; //added by Nima manually 6/18/2016

        for (b_i = 0; b_i < 4; b_i++) {
            xFormVec[b_i + 7 * (1 + i)] = dv0[b_i];
        }

        for (b_i = 0; b_i < 3; b_i++) {
            xFormVec[(b_i + 7 * (1 + i)) + 4] = dv2[b_i];
        }
    }

    //  2nd segment
    directKin(psi, 0.0, 2.0, dv3, homXformSeg1End);

    // hom. xform of the end of seg1 (top of end disk of seg1)
    for (i = 0; i < 4; i++) {
        Rbi_disk(theta2l, delta2, length, 2.0, 1.0 + (double)i, R_bi_disk);
        Pbt_disk(theta2l, delta2, length, 2.0, 1.0 + (double)i, dv2);
        for (b_i = 0; b_i < 3; b_i++) {
            for (i0 = 0; i0 < 3; i0++) {
                b_R_bi_disk[i0 + (b_i << 2)] = R_bi_disk[i0 + 3 * b_i];
            }
        }

        for (b_i = 0; b_i < 3; b_i++) {
            b_R_bi_disk[12 + b_i] = dv2[b_i];
        }

        for (b_i = 0; b_i < 4; b_i++) {
            b_R_bi_disk[3 + (b_i << 2)] = iv0[b_i];
        }

        for (b_i = 0; b_i < 4; b_i++) {
            for (i0 = 0; i0 < 4; i0++) {
                homXform[b_i + (i0 << 2)] = 0.0;
                for (i1 = 0; i1 < 4; i1++) {
                    homXform[b_i + (i0 << 2)] += homXformSeg1End[b_i + (i1 << 2)] *
                            b_R_bi_disk[i1 + (i0 << 2)];
                }
            }
        }

        for (b_i = 0; b_i < 3; b_i++) {
            for (i0 = 0; i0 < 3; i0++) {
                b_homXform[i0 + 3 * b_i] = homXform[i0 + (b_i << 2)];
            }
        }

        rot2Quat(b_homXform, dv0);
        b_i = 7 * (6 + i);
        for (i0 = 0; i0 < 4; i0++) {
            dv4[i0] = dv0[i0];
        }

        for (i0 = 0; i0 < 3; i0++) {
            dv4[i0 + 4] = homXform[12 + i0];
        }

        for (i0 = 0; i0 < 7; i0++) {
            xFormVec[i0 + b_i] = dv4[i0];
        }
    }

    //  3rd segment
    directKin(psi, 0.0, 3.0, dv3, homXformSeg1End);

    // hom. xform of the end of seg2 (top of end disk of seg2)
    for (i = 0; i < 3; i++) {
        Rbi_disk(theta3l, delta3, length, 3.0, 1.0 + (double)i, R_bi_disk);
        Pbt_disk(theta3l, delta3, length, 3.0, 1.0 + (double)i, dv2);
        for (b_i = 0; b_i < 3; b_i++) {
            for (i0 = 0; i0 < 3; i0++) {
                b_R_bi_disk[i0 + (b_i << 2)] = R_bi_disk[i0 + 3 * b_i];
            }
        }

        for (b_i = 0; b_i < 3; b_i++) {
            b_R_bi_disk[12 + b_i] = dv2[b_i];
        }

        for (b_i = 0; b_i < 4; b_i++) {
            b_R_bi_disk[3 + (b_i << 2)] = iv0[b_i];
        }

        for (b_i = 0; b_i < 4; b_i++) {
            for (i0 = 0; i0 < 4; i0++) {
                homXform[b_i + (i0 << 2)] = 0.0;
                for (i1 = 0; i1 < 4; i1++) {
                    homXform[b_i + (i0 << 2)] += homXformSeg1End[b_i + (i1 << 2)] *
                            b_R_bi_disk[i1 + (i0 << 2)];
                }
            }
        }

        for (b_i = 0; b_i < 3; b_i++) {
            for (i0 = 0; i0 < 3; i0++) {
                b_homXform[i0 + 3 * b_i] = homXform[i0 + (b_i << 2)];
            }
        }

        rot2Quat(b_homXform, dv0);
        b_i = 7 * (10 + i);
        for (i0 = 0; i0 < 4; i0++) {
            dv4[i0] = dv0[i0];
        }

        for (i0 = 0; i0 < 3; i0++) {
            dv4[i0 + 4] = homXform[12 + i0];
        }

        for (i0 = 0; i0 < 7; i0++) {
            xFormVec[i0 + b_i] = dv4[i0];
        }
    }
}


// Function Definitions

//
// parse constants
// Arguments    : double theta_tL
//                double delta_t
//                double segmentNum
//                double diskNum
//                double P_bt_disk[3]
// Return Type  : void
//
static void Pbt_disk(double theta_tL, double delta_t, const double length[3], double segmentNum, double
                     diskNum, double P_bt_disk[3])
{
    double Lt;
    int n_disk;
    double b_gamma;
    double rho;
    double dv5[9];
    double b_rho[9];
    int i3;
    static const signed char iv1[3] = { 0, 0, 1 };

    int i4;
    double dv6[3];
    //  This function determines the position of all the disks on a segment
    //  in the base disk frame (bt)
    //
    // first segment length
    // second segment length
    // third segment length
    // number of disks including end disk
    // number of disks including end disk
    // number of disks including end disk
    // disk height
    //
    if (segmentNum == 1.0) {
        Lt = length[0];
        n_disk = 5;
    } else if (segmentNum == 2.0) {
        Lt = length[1];
        n_disk = 4;
    } else {
        if (segmentNum == 3.0) {
            Lt = length[2];
            n_disk = 3;
        }
    }

    //
    if (fabs(theta_tL - 0.5*::PI) < 0.0017) {
        // almost straight
        P_bt_disk[0] = 0.0;
        P_bt_disk[1] = 0.0;
        P_bt_disk[2] = Lt - 0.5 * (1.0 + 4.0 * ((double)n_disk - diskNum)) * 3.2;
    } else {
        b_gamma = 0.5 * (1.0 + 4.0 * ((double)n_disk - diskNum)) * 3.2 *
                (0.5*PI - theta_tL) / Lt;

        // angle of the disk wrt end line
        rho = Lt / (0.5*PI - theta_tL);

        // segment radius
        // rotation of the bending plane wrt the base frame
        dv5[0] = cos(-delta_t);
        dv5[3] = -sin(-delta_t);
        dv5[6] = 0.0;
        dv5[1] = sin(-delta_t);
        dv5[4] = cos(-delta_t);
        dv5[7] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
            dv5[2 + 3 * i3] = iv1[i3];
            for (i4 = 0; i4 < 3; i4++) {
                b_rho[i4 + 3 * i3] = rho * dv5[i4 + 3 * i3];
            }
        }

        dv6[0] = 1.0 - cos((0.5*::PI - theta_tL) - b_gamma);
        dv6[1] = 0.0;
        dv6[2] = sin((0.5*::PI - theta_tL) - b_gamma);
        for (i3 = 0; i3 < 3; i3++) {
            P_bt_disk[i3] = 0.0;
            for (i4 = 0; i4 < 3; i4++) {
                P_bt_disk[i3] += b_rho[i3 + 3 * i4] * dv6[i4];
            }
        }
    }
}

//
// parse constants
// Arguments    : double theta_tL
//                double delta_t
//                double Lt
//                double P_bt_tl[3]
// Return Type  : void
//
static void Pbt_tl(double theta_tL, double delta_t, double Lt, double P_bt_tl[3])
{
    double y;

    //  parameters of the simulation
    //  Laset Edited 03/05/2016
    //
    // first segment length
    // 2nd segment length
    // 3rd segment length
    // secondary backbone length from A/U to 1st segment EE
    // secondary backbone length from A/U to 2nd segment EE
    // secondary backbone length from A/U to 3rd segment EE
    // first segment secondary backbone OD
    // first segment secondary backbone ID
    // second segment secondary backbone OD
    // second segment secondary backbone ID
    // third segment secondary backbone OD
    // third segment secondary backbone ID
    // end disk height
    // spacer disk height
    //  mbcrParam.D_PRIMARY = 0.3/1000;
    // primary elastic modulus (PTFE)
    // secondary elastic modulus (NiTi)
    // necessary for Matlab coder
    // number of disks including end disk
    // end function
    if (fabs(theta_tL - 0.5*PI) < 0.0017) {
        // almost straight
        P_bt_tl[0] = 0.0;
        P_bt_tl[1] = 0.0;
        P_bt_tl[2] = Lt;
    } else {
        y = Lt / (theta_tL - 0.5*PI);
        P_bt_tl[0] = -y * (cos(delta_t) * (1.0 - sin(theta_tL)));
        P_bt_tl[1] = -y * (sin(delta_t) * (sin(theta_tL) - 1.0));
        P_bt_tl[2] = -y * cos(theta_tL);
    }
}

//
// parse constants
// Arguments    : double theta_tL
//                double delta_t
//                double segmentNum
//                double diskNum
//                double R_bi_disk[9]
// Return Type  : void
//
static void Rbi_disk(double theta_tL, double delta_t, const double length[3], double segmentNum, double
                     diskNum, double R_bi_disk[9])
{
    double Lt;
    int n_disk;
    int k;
    double angle;
    double dv7[9];
    double dv8[9];
    int i5;
    static const signed char iv2[3] = { 0, 0, 1 };

    static const signed char iv3[3] = { 0, 1, 0 };

    double dv9[9];
    double dv10[9];
    int i6;
    int i7;

    //  This function determines the rotation of all the disks on a segment
    //  in the base disk frame (bt)
    //
    // first segment length
    // second segment length
    // third segment length
    // number of disks including end disk
    // number of disks including end disk
    // number of disks including end disk
    // disk height
    //
    if (segmentNum == 1.0) {
        Lt = length[0];
        n_disk = 5;
    } else if (segmentNum == 2.0) {
        Lt = length[1];
        n_disk = 4;
    } else {
        if (segmentNum == 3.0) {
            Lt = length[2];
            n_disk = 3;
        }
    }

    //
    if (fabs(theta_tL - 0.5*PI) < 0.0017) {
        // almost straight
        memset(&R_bi_disk[0], 0, 9U * sizeof(double));
        for (k = 0; k < 3; k++) {
            R_bi_disk[k + 3 * k] = 1.0;
        }
    } else {
        // angle of the disk wrt end line
        angle = (0.5*PI - theta_tL) - 0.5 * (1.0 + 4.0 * ((double)n_disk
                                                          - diskNum)) * 3.2 * (0.5*PI - theta_tL) / Lt;
        dv7[0] = cos(-delta_t);
        dv7[3] = -sin(-delta_t);
        dv7[6] = 0.0;
        dv7[1] = sin(-delta_t);
        dv7[4] = cos(-delta_t);
        dv7[7] = 0.0;
        dv8[0] = cos(angle);
        dv8[3] = 0.0;
        dv8[6] = sin(angle);
        for (i5 = 0; i5 < 3; i5++) {
            dv7[2 + 3 * i5] = iv2[i5];
            dv8[1 + 3 * i5] = iv3[i5];
        }

        dv8[2] = -sin(angle);
        dv8[5] = 0.0;
        dv8[8] = cos(angle);
        dv10[0] = cos(delta_t);
        dv10[3] = -sin(delta_t);
        dv10[6] = 0.0;
        dv10[1] = sin(delta_t);
        dv10[4] = cos(delta_t);
        dv10[7] = 0.0;
        for (i5 = 0; i5 < 3; i5++) {
            for (i6 = 0; i6 < 3; i6++) {
                dv9[i5 + 3 * i6] = 0.0;
                for (i7 = 0; i7 < 3; i7++) {
                    dv9[i5 + 3 * i6] += dv7[i5 + 3 * i7] * dv8[i7 + 3 * i6];
                }
            }

            dv10[2 + 3 * i5] = iv2[i5];
        }

        for (i5 = 0; i5 < 3; i5++) {
            for (i6 = 0; i6 < 3; i6++) {
                R_bi_disk[i5 + 3 * i6] = 0.0;
                for (i7 = 0; i7 < 3; i7++) {
                    R_bi_disk[i5 + 3 * i6] += dv9[i5 + 3 * i7] * dv10[i7 + 3 * i6];
                }
            }
        }
    }
}

//
// This function calculates direct kinematic homogenous transformation from
//  frame i to frame j where wcs is frame 0 and last frame is 4, the gripper
//  frame of the 3rd segment
// Arguments    : const double psi[7]
//                double frame_i
//                double frame_j
//                const double L[3]
//                double homTranij[16]
// Return Type  : void
//
// IMPORTANT: homTranij is an array that includes column-major hom. transfromation
void directKin(const double psi[7], double frame_i, double frame_j, const
double L[3], double homTranij[16])
{
    signed char R10[9];
    int k;
    double T[64];
    double dv11[3];
    double dv12[9];
    double dv13[9];
    static const signed char iv4[3] = { 0, 0, 1 };

    static const signed char iv5[3] = { 0, 1, 0 };

    double dv14[9];
    double dv15[9];
    int i8;
    int i9;
    static const signed char iv6[4] = { 0, 0, 0, 1 };

    double dv16[3];
    int p;
    int b_frame_i;
    double b_homTranij[16];
    int i10;

    //  parsing L
    for (k = 0; k < 9; k++) {
        R10[k] = 0;
    }

    for (k = 0; k < 3; k++) {
        R10[k + 3 * k] = 1;
    }

    //  parse constants
    // successive rotaiom matrix from segment t (b_t) to segment t+1 (gt or bt+1)
    //
    //  parse constants
    // successive rotaiom matrix from segment t (b_t) to segment t+1 (gt or bt+1)
    //
    //  parse constants
    // successive rotaiom matrix from segment t (b_t) to segment t+1 (gt or bt+1)
    //
    memset(&T[0], 0, sizeof(double) << 6);
    Pbt_tl(psi[4], psi[5], L[2], dv11);
    dv12[0] = cos(-psi[5]);
    dv12[3] = -sin(-psi[5]);
    dv12[6] = 0.0;
    dv12[1] = sin(-psi[5]);
    dv12[4] = cos(-psi[5]);
    dv12[7] = 0.0;
    dv13[0] = cos(0.5*::PI - psi[4]);
    dv13[3] = 0.0;
    dv13[6] = sin(0.5*::PI - psi[4]);
    for (k = 0; k < 3; k++) {
        dv12[2 + 3 * k] = iv4[k];
        dv13[1 + 3 * k] = iv5[k];
    }

    dv13[2] = -sin(0.5*PI - psi[4]);
    dv13[5] = 0.0;
    dv13[8] = cos(0.5*PI - psi[4]);
    dv15[0] = cos(psi[5]);
    dv15[3] = -sin(psi[5]);
    dv15[6] = 0.0;
    dv15[1] = sin(psi[5]);
    dv15[4] = cos(psi[5]);
    dv15[7] = 0.0;
    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            dv14[k + 3 * i8] = 0.0;
            for (i9 = 0; i9 < 3; i9++) {
                dv14[k + 3 * i8] += dv12[k + 3 * i9] * dv13[i9 + 3 * i8];
            }
        }

        dv15[2 + 3 * k] = iv4[k];
    }

    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            dv12[k + 3 * i8] = 0.0;
            for (i9 = 0; i9 < 3; i9++) {
                dv12[k + 3 * i8] += dv14[k + 3 * i9] * dv15[i9 + 3 * i8];
            }
        }
    }

    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            T[48 + (i8 + (k << 2))] = dv12[i8 + 3 * k];
        }
    }

    for (k = 0; k < 3; k++) {
        T[60 + k] = dv11[k];
    }

    for (k = 0; k < 4; k++) {
        T[51 + (k << 2)] = iv6[k];
    }

    Pbt_tl(psi[2], psi[3], L[1], dv11);
    dv12[0] = cos(-psi[3]);
    dv12[3] = -sin(-psi[3]);
    dv12[6] = 0.0;
    dv12[1] = sin(-psi[3]);
    dv12[4] = cos(-psi[3]);
    dv12[7] = 0.0;
    dv13[0] = cos(0.5*::PI - psi[2]);
    dv13[3] = 0.0;
    dv13[6] = sin(0.5*::PI - psi[2]);
    for (k = 0; k < 3; k++) {
        dv12[2 + 3 * k] = iv4[k];
        dv13[1 + 3 * k] = iv5[k];
    }

    dv13[2] = -sin(0.5*::PI - psi[2]);
    dv13[5] = 0.0;
    dv13[8] = cos(0.5*::PI - psi[2]);
    dv15[0] = cos(psi[3]);
    dv15[3] = -sin(psi[3]);
    dv15[6] = 0.0;
    dv15[1] = sin(psi[3]);
    dv15[4] = cos(psi[3]);
    dv15[7] = 0.0;
    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            dv14[k + 3 * i8] = 0.0;
            for (i9 = 0; i9 < 3; i9++) {
                dv14[k + 3 * i8] += dv12[k + 3 * i9] * dv13[i9 + 3 * i8];
            }
        }

        dv15[2 + 3 * k] = iv4[k];
    }

    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            dv12[k + 3 * i8] = 0.0;
            for (i9 = 0; i9 < 3; i9++) {
                dv12[k + 3 * i8] += dv14[k + 3 * i9] * dv15[i9 + 3 * i8];
            }
        }
    }

    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            T[32 + (i8 + (k << 2))] = dv12[i8 + 3 * k];
        }
    }

    for (k = 0; k < 3; k++) {
        T[44 + k] = dv11[k];
    }

    for (k = 0; k < 4; k++) {
        T[35 + (k << 2)] = iv6[k];
    }

    Pbt_tl(psi[0], psi[1], L[0], dv11);
    dv12[0] = cos(-psi[1]);
    dv12[3] = -sin(-psi[1]);
    dv12[6] = 0.0;
    dv12[1] = sin(-psi[1]);
    dv12[4] = cos(-psi[1]);
    dv12[7] = 0.0;
    dv13[0] = cos(0.5*::PI - psi[0]);
    dv13[3] = 0.0;
    dv13[6] = sin(0.5*::PI - psi[0]);
    for (k = 0; k < 3; k++) {
        dv12[2 + 3 * k] = iv4[k];
        dv13[1 + 3 * k] = iv5[k];
    }

    dv13[2] = -sin(0.5*::PI - psi[0]);
    dv13[5] = 0.0;
    dv13[8] = cos(0.5*::PI - psi[0]);
    dv15[0] = cos(psi[1]);
    dv15[3] = -sin(psi[1]);
    dv15[6] = 0.0;
    dv15[1] = sin(psi[1]);
    dv15[4] = cos(psi[1]);
    dv15[7] = 0.0;
    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            dv14[k + 3 * i8] = 0.0;
            for (i9 = 0; i9 < 3; i9++) {
                dv14[k + 3 * i8] += dv12[k + 3 * i9] * dv13[i9 + 3 * i8];
            }
        }

        dv15[2 + 3 * k] = iv4[k];
    }

    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            dv12[k + 3 * i8] = 0.0;
            for (i9 = 0; i9 < 3; i9++) {
                dv12[k + 3 * i8] += dv14[k + 3 * i9] * dv15[i9 + 3 * i8];
            }
        }
    }

    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            T[16 + (i8 + (k << 2))] = dv12[i8 + 3 * k];
        }
    }

    for (k = 0; k < 3; k++) {
        T[28 + k] = dv11[k];
    }

    for (k = 0; k < 4; k++) {
        T[19 + (k << 2)] = iv6[k];
    }

    dv16[0] = 0.0;
    dv16[1] = 0.0;
    dv16[2] = psi[6];
    for (k = 0; k < 3; k++) {
        for (i8 = 0; i8 < 3; i8++) {
            T[i8 + (k << 2)] = R10[i8 + 3 * k];
        }
    }

    for (k = 0; k < 3; k++) {
        T[12 + k] = dv16[k];
    }

    for (k = 0; k < 4; k++) {
        T[3 + (k << 2)] = iv6[k];
        for (i8 = 0; i8 < 4; i8++) {
            homTranij[i8 + (k << 2)] = T[(i8 + (k << 2)) + (((int)(frame_i + 1.0) - 1)
                                                            << 4)];
        }
    }

    if (frame_i + 1.0 != frame_j) {
        k = (int)(frame_j + (1.0 - ((frame_i + 1.0) + 1.0)));
        for (p = 0; p < k; p++) {
            b_frame_i = (int)(((frame_i + 1.0) + 1.0) + (double)p);
            for (i8 = 0; i8 < 4; i8++) {
                for (i9 = 0; i9 < 4; i9++) {
                    b_homTranij[i8 + (i9 << 2)] = 0.0;
                    for (i10 = 0; i10 < 4; i10++) {
                        b_homTranij[i8 + (i9 << 2)] += homTranij[i8 + (i10 << 2)] * T[(i10 +
                                                                                       (i9 << 2)) + ((b_frame_i - 1) << 4)];
                    }
                }
            }

            for (i8 = 0; i8 < 4; i8++) {
                for (i9 = 0; i9 < 4; i9++) {
                    homTranij[i9 + (i8 << 2)] = b_homTranij[i9 + (i8 << 2)];
                }
            }
        }
    }
}

//
// Arguments    : const double R[9]
//                double q[4]
// Return Type  : void
//
static void rot2Quat(const double R[9], double q[4])
{
    double x;
    double b_x;
    double c_x;
    double d_x;
    int e_x;
    double b_R;
    double s1[3];
    double f_x;
    int g_x;
    double c_R;
    double h_x;
    int i_x;
    double d_R;
    int i2;

    // % rot2quat: Rotation matrix to quaternion
    //  This function converts a rotation matrix R into a normalized quaternion
    //  q. Taken from Siciliano's textbook
    //
    //  Author: Jason Pile
    //
    x = R[5] - R[7];
    b_x = R[6] - R[2];
    c_x = R[1] - R[3];
    if (x < 0.0) {
        d_x = -1.0;
    } else if (x > 0.0) {
        d_x = 1.0;
    } else {
        d_x = x;
    }

    if (d_x >= 0.0) {
        e_x = 1;
    } else {
        e_x = -1;
    }

    if (((R[0] - R[4]) - R[8]) + 1.0 >= 0.0) {
        b_R = ((R[0] - R[4]) - R[8]) + 1.0;
    } else {
        b_R = 0.0;
    }

    s1[0] = (double)e_x * sqrt(b_R);
    if (b_x < 0.0) {
        f_x = -1.0;
    } else if (b_x > 0.0) {
        f_x = 1.0;
    } else {
        f_x = b_x;
    }

    if (f_x >= 0.0) {
        g_x = 1;
    } else {
        g_x = -1;
    }

    if (((R[4] - R[0]) - R[8]) + 1.0 >= 0.0) {
        c_R = ((R[4] - R[0]) - R[8]) + 1.0;
    } else {
        c_R = 0.0;
    }

    s1[1] = (double)g_x * sqrt(c_R);
    if (c_x < 0.0) {
        h_x = -1.0;
    } else if (c_x > 0.0) {
        h_x = 1.0;
    } else {
        h_x = c_x;
    }

    if (h_x >= 0.0) {
        i_x = 1;
    } else {
        i_x = -1;
    }

    if (((R[8] - R[4]) - R[0]) + 1.0 >= 0.0) {
        d_R = ((R[8] - R[4]) - R[0]) + 1.0;
    } else {
        d_R = 0.0;
    }

    s1[2] = (double)i_x * sqrt(d_R);
    q[0] = sqrt((((R[0] + R[4]) + R[8]) + 1.0) / 4.0);
    for (i2 = 0; i2 < 3; i2++) {
        q[i2 + 1] = 0.5 * s1[i2];
    }
}


//
// File trailer for get_robot_xform.cpp
//
// [EOF]
//
