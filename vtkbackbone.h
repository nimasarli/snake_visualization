#ifndef VTKBACKBONE_H
#define VTKBACKBONE_H

#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkLineSource.h>
#include <vtkArcSource.h>
//#include <vtkMapper.h>
#include <vtkActor.h>
//#include <vtkRenderer.h>
#include <vtkTransform.h>
#include "robotic.h"
#include "continuum_robot.h"
#include "globals.h"
#include <cmath>
#include <vtkTransform.h>
#include <Eigen/Core>
#include <vtkMatrix4x4.h>

class VtkBackbone
{
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW // takes care of alignment issues (necessaty
    // when using new operator of a class that has fixed size vectorizable Eigen
    // object)
    VtkBackbone(continuum_robot::ConfigSingleSeg config,
                continuum_robot::BackboneNum backboneNum,
                continuum_robot::MbcrParameter mbcrParameter);
    void setTransform(const Eigen::Matrix4d &xform); //set rigid xform of backbone
    void genShapeMapper(continuum_robot::ConfigSingleSeg config); //generates mapper for shape of backbone
    void setPrimBackboneLength(double length);
    void setBackboneKinRadius(double kinRadius);
    void setBackboneNumber(continuum_robot::BackboneNum backboneNum);
    double getPrimBackboneLength();
    double getBackboneKinRadius();
    continuum_robot::BackboneNum getBackboneNumber();
    vtkSmartPointer<vtkActor> getActor();

private:
    vtkSmartPointer<vtkLineSource> lineSourcePtr_;
    vtkSmartPointer<vtkArcSource> arcSourcePtr_;
    vtkSmartPointer<vtkPolyDataMapper> mapperPtr_;
    vtkSmartPointer<vtkActor> actorPtr_;
    vtkSmartPointer<vtkMatrix4x4> xformVtkMatPtr_;
    vtkSmartPointer<vtkTransform> transformPtr_;
    continuum_robot::MbcrParameter mbcrParameter_;
    continuum_robot::BackboneNum backboneNum_;

};


#endif // VTKBACKBONE_H
