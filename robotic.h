/* Declaration file for robotic namespace
 * Author: Nima Sarli
 */
#ifndef ROBOTIC
#define ROBOTIC

#include <math.h>
#include <vector>
#include <Eigen/Core>

namespace robotic {

/****************************************************************************
 *  Data Structures
 ****************************************************************************/
//
//struct PositionArray
//{
//    double position[3];
//};

//
//struct RotationMatrix
//{
//    double rotation_mat[3][3];
//};

//
//struct Quaternion
//{
//    double quaternion[4];
//};

/****************************************************************************
 *  Functions
 ****************************************************************************/

template <typename T>
Eigen::Matrix3d quat2Rot(const Eigen::MatrixBase<T> &quaternion)
{

    double a = quaternion(0); //q0
    double b = quaternion(1); //qx
    double c = quaternion(2); //qy
    double d = quaternion(3); //qz

    Eigen::Matrix3d rotMat;
    rotMat << pow(a,2)+pow(b,2)-pow(c,2)-pow(d,2), 2*(b*c-a*d), 2*(b*d+a*c),
            2*(b*c+a*d), pow(a,2)-pow(b,2)+pow(c,2)-pow(d,2), 2*(c*d-a*b),
            2*(b*d-a*c), 2*(c*d+a*b), pow(a,2)-pow(b,2)-pow(c,2)+pow(d,2);
    return rotMat;
}

//
Eigen::Matrix3d rotX(double angle);

//
Eigen::Matrix3d rotY(double angle);

//
Eigen::Matrix3d rotZ(double angle);



}

#endif // ROBOTIC

