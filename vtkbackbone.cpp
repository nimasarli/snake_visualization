#include "vtkbackbone.h"
#include <Eigen/Core>
#include <vtkLineSource.h>
#include <vtkArcSource.h>

VtkBackbone::VtkBackbone(continuum_robot::ConfigSingleSeg config,
                         continuum_robot::BackboneNum backboneNum,
                         continuum_robot::MbcrParameter mbcrParameter): mbcrParameter_(mbcrParameter),
    backboneNum_(backboneNum)
{

    // generate mapper
    mapperPtr_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    lineSourcePtr_ = vtkSmartPointer<vtkLineSource>::New();
    arcSourcePtr_ = vtkSmartPointer<vtkArcSource>::New();
    genShapeMapper(config); //generate backbone shape

    // set mapper
    actorPtr_ = vtkSmartPointer<vtkActor>::New();
    actorPtr_->SetMapper(mapperPtr_);

    // set transformations
    transformPtr_ = vtkSmartPointer<vtkTransform>::New();
    transformPtr_->Identity();
    actorPtr_->SetUserTransform(transformPtr_);

    // initialize xformVtkMatPtr_
    xformVtkMatPtr_ = vtkSmartPointer<vtkMatrix4x4>::New();
    xformVtkMatPtr_->Identity();

    //
    //pArcSource->SetOutputPointsPrecision(vtkAlgorithm::SINGLE_PRECISION);

    //    pArcSource->SetNormal(0,0,1); //plane normal
    //    pArcSource->SetPolarVector(1,90,0); // polar coordinate (r,theta,z) of starting point
    //    pArcSource->SetAngle(180); //CCW angle of arc
    //    pArcSource->Update();
}

vtkSmartPointer<vtkActor> VtkBackbone::getActor()
{
    return actorPtr_;
}

void VtkBackbone::setTransform(const Eigen::Matrix4d &xform)
{
    // update hom transform of actor
    // note position must be in mm
    double homXform[16] = {xform(0,0), xform(0,1), xform(0,2), xform(0,3),
                           xform(1,0), xform(1,1), xform(1,2), xform(1,3),
                           xform(2,0), xform(2,1), xform(2,2), xform(2,3),
                           xform(3,0), xform(3,1), xform(3,2), xform(3,3)};
    xformVtkMatPtr_->DeepCopy(homXform);
    transformPtr_->SetMatrix(xformVtkMatPtr_);
    transformPtr_->Update();
}

void VtkBackbone::genShapeMapper(continuum_robot::ConfigSingleSeg config)
{
    //config = flipConfig(config);
    Eigen::Vector3d pointStart;
    switch (backboneNum_)
    {
    case continuum_robot::BACKBONE1:
        pointStart << mbcrParameter_.kin_Radius,
                0,
                0;
        break;
    case continuum_robot::BACKBONE2:
        pointStart << -mbcrParameter_.kin_Radius*(1.0/2.0),
                mbcrParameter_.kin_Radius*(std::sqrt(3.0)/2.0),
                0;
        break;
    case continuum_robot::BACKBONE3:
        pointStart << -mbcrParameter_.kin_Radius*(1.0/2.0),
                -mbcrParameter_.kin_Radius*(std::sqrt(3.0)/2.0),
                0;
        break;
    }

    Eigen::Matrix4d gripperToBaseXform = continuum_robot::directKinSingleSeg(config,
                                                                             mbcrParameter_.length);
    Eigen::Vector3d pointEnd = gripperToBaseXform.block(0,0,3,3)*pointStart + gripperToBaseXform.block(0,3,3,1);

    if (abs(config.theta-::PI/2.0) <= 0.5*(::PI/180.0))
    {
        lineSourcePtr_->SetPoint1(pointStart(0),pointStart(1),pointStart(2));
        lineSourcePtr_->SetPoint2(pointEnd(0),pointEnd(1),pointEnd(2));
        lineSourcePtr_->Update();
        mapperPtr_->SetInputConnection(lineSourcePtr_->GetOutputPort());
    }
    else
    {
        Eigen::Vector3d pointCenter = continuum_robot::getBackboneCurveCenterSingleSeg(config,
                                                                                       mbcrParameter_,
                                                                                       backboneNum_);
        arcSourcePtr_->SetResolution(50);
        arcSourcePtr_->SetPoint1(pointStart(0),pointStart(1),pointStart(2));
        arcSourcePtr_->SetPoint2(pointEnd(0),pointEnd(1),pointEnd(2));
        arcSourcePtr_->SetCenter(pointCenter(0),pointCenter(1),pointCenter(2));
        arcSourcePtr_->Update();
        mapperPtr_->SetInputConnection(arcSourcePtr_->GetOutputPort());
    }
}

void VtkBackbone::setPrimBackboneLength(double length)
{
    mbcrParameter_.length = length;
}

void VtkBackbone::setBackboneKinRadius(double kinRadius)
{
    mbcrParameter_.kin_Radius = kinRadius;
}

void VtkBackbone::setBackboneNumber(continuum_robot::BackboneNum backboneNum)
{
    backboneNum_ = backboneNum;
}

double VtkBackbone::getPrimBackboneLength()
{
    return mbcrParameter_.length;
}

double VtkBackbone::getBackboneKinRadius()
{
    return mbcrParameter_.kin_Radius;
}

continuum_robot::BackboneNum VtkBackbone::getBackboneNumber()
{
    return backboneNum_;
}

