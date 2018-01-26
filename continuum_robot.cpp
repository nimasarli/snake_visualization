#include "continuum_robot.h"

Eigen::Vector3d continuum_robot::directKinSingleSegPosition(continuum_robot::ConfigSingleSeg config,
                                                            double length)
{     
    double theta = config.theta;
    double delta = config.delta;
    double delTheta = theta - ::PI/2.0;
    Eigen::Vector3d position;

    if (std::abs(delTheta) <= 0.2*(::PI/180.0)) //less than 0.2 degree
    {
        position(0) = 0.0;
        position(1) = 0.0;
        position(2) = length;
    }
    else
    {
        position(0) = -(length/delTheta)*std::cos(delta)*(1-std::sin(theta));
        position(1) = +(length/delTheta)*std::sin(delta)*(1-std::sin(theta));
        position(2) = -(length/delTheta)*std::cos(theta);
    }

    return position;
}

Eigen::Matrix3d continuum_robot::directKinSingleSegRot(continuum_robot::ConfigSingleSeg config)
{
    //    rotMat = rotZ(-delta_t)*rotY(THETA_0-theta_tL)*rotZ(delta_t);
    double theta = config.theta;
    double delta = config.delta;

    //Eigen::Matrix3d rotMat = robotic::rotZ(-delta) * robotic::rotY(theta - ::PI/2.0) * robotic::rotZ(delta);
    Eigen::Matrix3d rotMat = robotic::rotZ(-delta) * robotic::rotY(::PI/2.0 - theta) * robotic::rotZ(delta);

    return rotMat;
}

Eigen::Matrix4d continuum_robot::directKinSingleSeg(continuum_robot::ConfigSingleSeg config, double length)
{
    Eigen::Vector3d position = directKinSingleSegPosition(config,length);
    Eigen::Matrix3d rotMat = directKinSingleSegRot(config);
    Eigen::Matrix4d xForm;
    xForm << rotMat, position,
            0, 0, 0, 1;

    return xForm;
}

Eigen::Vector3d continuum_robot::getBackboneLengthVecSingleSeg(continuum_robot::ConfigSingleSeg config,
                                                               continuum_robot::MbcrParameter mbcrParameter)
{
    Eigen::Vector3d backboneLengthVec;
    backboneLengthVec(0) = mbcrParameter.length + mbcrParameter.kin_Radius*std::cos(config.delta)*(config.theta-::PI/2.0);
    backboneLengthVec(1) = mbcrParameter.length + mbcrParameter.kin_Radius*std::cos(config.delta+(2.0/3.0)*::PI)*(config.theta-::PI/2.0);
    backboneLengthVec(2) = mbcrParameter.length + mbcrParameter.kin_Radius*std::cos(config.delta+(4.0/3.0)*::PI)*(config.theta-::PI/2.0);

    return backboneLengthVec;
}

double continuum_robot::getBackboneLengthSingleSeg(continuum_robot::ConfigSingleSeg config,
                                                   continuum_robot::MbcrParameter mbcrParameter,
                                                   continuum_robot::BackboneNum backboneNum)
{
    double backboneLength;
    switch (backboneNum)
    {
    case BACKBONE1:
        backboneLength = mbcrParameter.length + mbcrParameter.kin_Radius*std::cos(config.delta)*(config.theta-::PI/2.0);
        break;
    case BACKBONE2:
        backboneLength = mbcrParameter.length + mbcrParameter.kin_Radius*std::cos(config.delta+(2.0/3.0)*::PI)*(config.theta-::PI/2.0);
        break;
    case BACKBONE3:
        backboneLength = mbcrParameter.length + mbcrParameter.kin_Radius*std::cos(config.delta+(4.0/3.0)*::PI)*(config.theta-::PI/2.0);
        break;
    }

    return backboneLength;
}

// Find center of backbone curves. See doc/backbones_curve_center.pdf
Eigen::Vector3d continuum_robot::getBackboneCurveCenterSingleSeg(continuum_robot::ConfigSingleSeg config, continuum_robot::MbcrParameter mbcrParameter, continuum_robot::BackboneNum backboneNum)
{
    double bbLength = getBackboneLengthSingleSeg(config,mbcrParameter,backboneNum);
    double arcRadius = bbLength / abs(::PI/2-config.theta);
    ConfigSingleSeg configFlipped = flipConfig(config);
    Eigen::Vector3d curveCenter;
    curveCenter << mbcrParameter.kin_Radius*std::cos((backboneNum-1)*(2.0*::PI/3.0)) + arcRadius*std::cos(configFlipped.delta),
            mbcrParameter.kin_Radius*std::sin((backboneNum-1)*(2.0*::PI/3.0)) - arcRadius*std::sin(configFlipped.delta),
            0;

    return curveCenter;
}

continuum_robot::ConfigSingleSeg continuum_robot::flipConfig(continuum_robot::ConfigSingleSeg config)
{
    continuum_robot::ConfigSingleSeg configFlipped = config;
    if (config.theta > ::PI/2.0)
    {
        configFlipped.delta = ::PI + config.delta;
        configFlipped.theta = ::PI - config.theta;
    }
    return configFlipped;
}
