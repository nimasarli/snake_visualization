#ifndef VTKMBCROBOT_H
#define VTKMBCROBOT_H

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include "vtksolid.h"
#include "vtkbackbone.h"
#include "get_robot_xform.h"
#include "continuum_robot.h"
#include <memory>
#include <array>

class VtkMBCRobot
{
public:
    VtkMBCRobot();
    std::vector<double> getRobotXform(const double psi[7]);
    vtkSmartPointer<vtkRenderer> getRenderer();
    void setRobotXform(double psi[7]);

private:
    double xFormVec_[91];
    vtkSmartPointer<vtkRenderer> renderer_;
    std::shared_ptr<VtkSolid> disk1_seg1_;
    std::shared_ptr<VtkSolid> disk2_seg1_;
    std::shared_ptr<VtkSolid> disk3_seg1_;
    std::shared_ptr<VtkSolid> disk4_seg1_;
    std::shared_ptr<VtkSolid> disk5_seg1_;
    std::shared_ptr<VtkSolid> disk1_seg2_;
    std::shared_ptr<VtkSolid> disk2_seg2_;
    std::shared_ptr<VtkSolid> disk3_seg2_;
    std::shared_ptr<VtkSolid> disk4_seg2_;
    std::shared_ptr<VtkSolid> disk1_seg3_;
    std::shared_ptr<VtkSolid> disk2_seg3_;
    std::shared_ptr<VtkSolid> disk3_seg3_;
    std::shared_ptr<VtkBackbone> backbone1Seg1Ptr_;
    std::shared_ptr<VtkBackbone> backbone2Seg1Ptr_;
    std::shared_ptr<VtkBackbone> backbone3Seg1Ptr_;
    std::shared_ptr<VtkBackbone> backbone1Seg2Ptr_;
    std::shared_ptr<VtkBackbone> backbone2Seg2Ptr_;
    std::shared_ptr<VtkBackbone> backbone3Seg2Ptr_;
    std::shared_ptr<VtkBackbone> backbone1Seg3Ptr_;
    std::shared_ptr<VtkBackbone> backbone2Seg3Ptr_;
    std::shared_ptr<VtkBackbone> backbone3Seg3Ptr_;
    double kinRadius_;
    //std::vector<double> length_(3);
    std::array<double,3> length_;

};

#endif // VTKMBCROBOT_H
