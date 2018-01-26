#include "globals.h"
#include "vtksolid.h"
#include <robotic.h>

VtkSolid::VtkSolid(const char *path)
{
    // initialize solid pose to Identity matrix
    quaternion_(0) = 1.0;
    quaternion_(1) = 0.0;
    quaternion_(2) = 0.0;
    quaternion_(3) = 0.0;

    // Initialize the vtk objects
    actor_ = vtkSmartPointer<vtkActor>::New();
    stlReader_ = vtkSmartPointer<vtkSTLReader>::New();
    mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    objReader_ = vtkSmartPointer<vtkOBJReader>::New();

    // Read the stl model file
    stlReader_->SetFileName(path);
    // objReader->SetFileName("models/polaris_vicra.obj");

    // build the vtk pipeline
    mapper_->SetInputConnection(stlReader_->GetOutputPort());
    actor_->SetMapper(mapper_);

    // Setup Transformations
    T_ = vtkSmartPointer<vtkTransform>::New();
    T_->Identity();
    actor_->SetUserTransform(T_);

}

// returns pointer to the actor object so that other objects may add this solid object to a scene
vtkSmartPointer<vtkActor> VtkSolid::getActor()
{
    return actor_;
}

void VtkSolid::setPose(double pose[7])
{
    // pose = [quat,position] //position is in m
    // quat = [cos(angle/2) sin(angle/2)*rx sin(angle/2)*ry sin(angle/2)*rz]
    // extract quaternions and convert to axis angle

    // convert quaternion to rotation matrix
    quaternion_(0) = pose[0];
    quaternion_(1) = pose[1];
    quaternion_(2) = pose[2];
    quaternion_(3) = pose[3];
    rotationMat_ = robotic::quat2Rot(quaternion_);

    // update T
    poseXform_ = vtkSmartPointer<vtkMatrix4x4>::New();
    double xform[16] = {rotationMat_(0,0), rotationMat_(0,1), rotationMat_(0,2), pose[4],
                           rotationMat_(1,0), rotationMat_(1,1), rotationMat_(1,2), pose[5],
                           rotationMat_(2,0), rotationMat_(2,1), rotationMat_(2,2), pose[6],
                           0.0, 0.0, 0.0, 1.0};
    poseXform_->DeepCopy(xform);
    T_->SetMatrix(poseXform_);
    T_->Update();

}

