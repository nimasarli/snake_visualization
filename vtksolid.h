#ifndef VTKSOLID_H
#define VTKSOLID_H

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkSTLReader.h>
#include <vtkTransform.h>
#include <math.h>
#include <vtkOBJReader.h>
#include <vtkMatrix4x4.h>
#include <Eigen/Core>

class VtkSolid
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // takes care of alignment issues (necessaty
    // when using new operator of a class that has fixed size vectorizable Eigen
    // object)
    VtkSolid(const char *path);
    vtkSmartPointer<vtkActor> getActor();
    void setPose(double pose[]);

private:
    vtkSmartPointer<vtkActor> actor_;
    vtkSmartPointer<vtkSTLReader> stlReader_;
    vtkSmartPointer<vtkOBJReader> objReader_;
    vtkSmartPointer<vtkPolyDataMapper> mapper_;
    vtkSmartPointer<vtkTransform> T_;
    vtkSmartPointer<vtkMatrix4x4> poseXform_;
    Eigen::Matrix3d rotationMat_;
    Eigen::Vector4d quaternion_;

};

#endif // VTKSOLID_H
