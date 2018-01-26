/* Implentation file for robotic namespace
 * Author: Nima Sarli
 */
#include "robotic.h"

/****************************************************************************
 *  Functions
 ****************************************************************************/

//
Eigen::Matrix3d robotic::rotX(double angle)
{
    Eigen::Matrix3d rotMat;
    rotMat << 1.0, 0.0, 0.0,
            0.0, std::cos(angle), -std::sin(angle),
            0.0, std::sin(angle), std::cos(angle);
    return rotMat;
}

//
Eigen::Matrix3d robotic::rotY(double angle)
{
    Eigen::Matrix3d rotMat;
    rotMat << std::cos(angle), 0, std::sin(angle),
            0.0, 1.0, 0.0,
            -std::sin(angle), 0.0, std::cos(angle);
    return rotMat;
}

//
Eigen::Matrix3d robotic::rotZ(double angle)
{
    Eigen::Matrix3d rotMat;
    rotMat << std::cos(angle), -std::sin(angle), 0.0,
            std::sin(angle), std::cos(angle), 0.0,
            0.0, 0.0, 1.0;
    return rotMat;
}
