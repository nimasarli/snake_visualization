/* Declaration file for continuum robot namespace
 * Author: Nima Sarli
 */

#ifndef CONTINUUM_ROBOT
#define CONTINUUM_ROBOT

#include <Eigen/Core>
#include "globals.h"
#include "robotic.h"

namespace continuum_robot
{
/****************************************************************************
 *  Data Structures
 ****************************************************************************/
//
struct ConfigSingleSeg {
    double theta;
    double delta;
};

//
struct MbcrParameter
{
    double length;
    double kin_Radius;
};

/****************************************************************************
 *  enums
 ****************************************************************************/
//
enum BackboneNum {
    BACKBONE1 = 1,
    BACKBONE2 = 2,
    BACKBONE3 = 3
};

/****************************************************************************
 *  Functions
 ****************************************************************************/
//
Eigen::Vector3d directKinSingleSegPosition(ConfigSingleSeg config,
                                           double length);
//
Eigen::Matrix3d directKinSingleSegRot(ConfigSingleSeg config);

//
Eigen::Matrix4d directKinSingleSeg(ConfigSingleSeg config,
                                   double length);

//
Eigen::Vector3d getBackboneLengthVecSingleSeg(ConfigSingleSeg config,
                                              MbcrParameter mbcrParameter);

//
double getBackboneLengthSingleSeg(ConfigSingleSeg config,
                                  MbcrParameter mbcrParameter,
                                  BackboneNum backboneNum);

//
Eigen::Vector3d getBackboneCurveCenterSingleSeg(ConfigSingleSeg config,
                                       MbcrParameter mbcrParameter,
                                       BackboneNum backboneNum);
// Find center of backbone curves. See doc/backbones_curve_center.pdf

//
ConfigSingleSeg flipConfig(ConfigSingleSeg config);

}
#endif // CONTINUUM_ROBOT

