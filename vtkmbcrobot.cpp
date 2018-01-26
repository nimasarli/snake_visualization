#include "vtkmbcrobot.h"
#include <stdlib.h>
#include <vector>
#include <vtkProperty.h>
#include "globals.h"
#include <Eigen/Core>
#include "get_robot_xform.h"

VtkMBCRobot::VtkMBCRobot(): length_({{32.0,25.0,15.5}}), // this is aggregate initialization. single braces may not work on some compilers
    kinRadius_(1.75)
{

    // Disks
    disk1_seg1_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk2_seg1_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk3_seg1_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk4_seg1_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk5_seg1_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk1_seg2_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk2_seg2_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk3_seg2_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk4_seg2_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk1_seg3_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk2_seg3_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));
    disk3_seg3_ = std::shared_ptr <VtkSolid> (new VtkSolid("G:/DEV/Qt_projects/qt_5_5_1_msvc_2013/TURBT_TeleOp_Simulation.git/"
                                                           "TURBT_TeleOp_Simulation/model/disk.stl"));

    //
    double psiHome[] = {::PI/2.0,0,::PI/2.0,0,::PI/2.0,0,0};
    double lengthArr[] = {length_[0],length_[1],length_[2]};

    // segment1
    continuum_robot::ConfigSingleSeg seg1Config;
    continuum_robot::MbcrParameter seg1Parameter;
    seg1Config.theta = psiHome[0];
    seg1Config.delta = psiHome[1];
    seg1Parameter.kin_Radius = kinRadius_;
    seg1Parameter.length = length_[0];
    backbone1Seg1Ptr_ = std::shared_ptr<VtkBackbone>(new VtkBackbone(seg1Config,
                                                                     continuum_robot::BACKBONE1,
                                                                     seg1Parameter));
    backbone2Seg1Ptr_ = std::shared_ptr<VtkBackbone>(new VtkBackbone(seg1Config,
                                                                     continuum_robot::BACKBONE2,
                                                                     seg1Parameter));
    backbone3Seg1Ptr_ = std::shared_ptr<VtkBackbone>(new VtkBackbone(seg1Config,
                                                                     continuum_robot::BACKBONE3,
                                                                     seg1Parameter));
    // segment2
    continuum_robot::ConfigSingleSeg seg2Config;
    continuum_robot::MbcrParameter seg2Parameter;
    seg2Config.theta = psiHome[2];
    seg2Config.delta = psiHome[3];
    seg2Parameter.kin_Radius = kinRadius_;
    seg2Parameter.length = length_[1];
    backbone1Seg2Ptr_ = std::shared_ptr<VtkBackbone>(new VtkBackbone(seg2Config,
                                                                     continuum_robot::BACKBONE1,
                                                                     seg2Parameter));
    backbone2Seg2Ptr_ = std::shared_ptr<VtkBackbone>(new VtkBackbone(seg2Config,
                                                                     continuum_robot::BACKBONE2,
                                                                     seg2Parameter));
    backbone3Seg2Ptr_ = std::shared_ptr<VtkBackbone>(new VtkBackbone(seg2Config,
                                                                     continuum_robot::BACKBONE3,
                                                                     seg2Parameter));
    // segment3
    continuum_robot::ConfigSingleSeg seg3Config;
    continuum_robot::MbcrParameter seg3Parameter;
    seg3Config.theta = psiHome[4];
    seg3Config.delta = psiHome[5];
    seg3Parameter.kin_Radius = kinRadius_;
    seg3Parameter.length = length_[2];
    backbone1Seg3Ptr_ = std::shared_ptr<VtkBackbone>(new VtkBackbone(seg3Config,
                                                                     continuum_robot::BACKBONE1,
                                                                     seg3Parameter));
    backbone2Seg3Ptr_ = std::shared_ptr<VtkBackbone>(new VtkBackbone(seg3Config,
                                                                     continuum_robot::BACKBONE2,
                                                                     seg3Parameter));
    backbone3Seg3Ptr_ = std::shared_ptr<VtkBackbone>(new VtkBackbone(seg3Config,
                                                                     continuum_robot::BACKBONE3,
                                                                     seg3Parameter));

    // set frames of segment1 backbones
    double frameI = 0;
    double frameJ = 1;
    double homXform01[16];
    directKin(psiHome, frameI, frameJ, lengthArr, homXform01);
    Eigen::Matrix4d xform;
    xform << homXform01[0],homXform01[4],homXform01[8],homXform01[12],
            homXform01[1],homXform01[5],homXform01[9],homXform01[13],
            homXform01[2],homXform01[6],homXform01[10],homXform01[14],
            homXform01[3],homXform01[7],homXform01[11],homXform01[15];
    backbone1Seg1Ptr_->setTransform(xform);
    backbone2Seg1Ptr_->setTransform(xform);
    backbone3Seg1Ptr_->setTransform(xform);

    // set frames of segment2 backbones
    frameJ = 2;
    double homXform02[16];
    directKin(psiHome, frameI, frameJ, lengthArr, homXform02);
    xform << homXform02[0],homXform02[4],homXform02[8],homXform02[12],
            homXform02[1],homXform02[5],homXform02[9],homXform02[13],
            homXform02[2],homXform02[6],homXform02[10],homXform02[14],
            homXform02[3],homXform02[7],homXform02[11],homXform02[15];
    backbone1Seg2Ptr_->setTransform(xform);
    backbone2Seg2Ptr_->setTransform(xform);
    backbone3Seg2Ptr_->setTransform(xform);

    // set frames of segment3 backbones
    frameJ = 3;
    double homXform03[16];
    directKin(psiHome, frameI, frameJ, lengthArr, homXform03);
    xform << homXform03[0],homXform03[4],homXform03[8],homXform03[12],
            homXform03[1],homXform03[5],homXform03[9],homXform03[13],
            homXform03[2],homXform03[6],homXform03[10],homXform03[14],
            homXform03[3],homXform03[7],homXform03[11],homXform03[15];
    backbone1Seg3Ptr_->setTransform(xform);
    backbone2Seg3Ptr_->setTransform(xform);
    backbone3Seg3Ptr_->setTransform(xform);

    //
    renderer_ = vtkSmartPointer<vtkRenderer>::New();

    renderer_->AddActor(disk1_seg1_->getActor());
    renderer_->AddActor(disk2_seg1_->getActor());
    renderer_->AddActor(disk3_seg1_->getActor());
    renderer_->AddActor(disk4_seg1_->getActor());
    renderer_->AddActor(disk5_seg1_->getActor());
    renderer_->AddActor(disk1_seg2_->getActor());
    renderer_->AddActor(disk2_seg2_->getActor());
    renderer_->AddActor(disk3_seg2_->getActor());
    renderer_->AddActor(disk4_seg2_->getActor());
    renderer_->AddActor(disk1_seg3_->getActor());
    renderer_->AddActor(disk2_seg3_->getActor());
    renderer_->AddActor(disk3_seg3_->getActor());
    renderer_->AddActor(backbone1Seg1Ptr_->getActor());
    renderer_->AddActor(backbone2Seg1Ptr_->getActor());
    renderer_->AddActor(backbone3Seg1Ptr_->getActor());
    renderer_->AddActor(backbone1Seg2Ptr_->getActor());
    renderer_->AddActor(backbone2Seg2Ptr_->getActor());
    renderer_->AddActor(backbone3Seg2Ptr_->getActor());
    renderer_->AddActor(backbone1Seg3Ptr_->getActor());
    renderer_->AddActor(backbone2Seg3Ptr_->getActor());
    renderer_->AddActor(backbone3Seg3Ptr_->getActor());

    disk1_seg1_->getActor()->GetProperty()->SetColor(0.6,0.6,0.6);
    disk2_seg1_->getActor()->GetProperty()->SetColor(0.6,0.6,0.6);
    disk3_seg1_->getActor()->GetProperty()->SetColor(0.6,0.6,0.6);
    disk4_seg1_->getActor()->GetProperty()->SetColor(0.6,0.6,0.6);
    disk5_seg1_->getActor()->GetProperty()->SetColor(0.0,1.0,0.0);
    disk1_seg2_->getActor()->GetProperty()->SetColor(0.6,0.6,0.6);
    disk2_seg2_->getActor()->GetProperty()->SetColor(0.6,0.6,0.6);
    disk3_seg2_->getActor()->GetProperty()->SetColor(0.6,0.6,0.6);
    disk4_seg2_->getActor()->GetProperty()->SetColor(0.0,1.0,0.0);
    disk1_seg3_->getActor()->GetProperty()->SetColor(0.6,0.6,0.6);
    disk2_seg3_->getActor()->GetProperty()->SetColor(0.6,0.6,0.6);
    disk3_seg3_->getActor()->GetProperty()->SetColor(0.0,1.0,0.0);
    backbone1Seg1Ptr_->getActor()->GetProperty()->SetColor(1,0,0);
    backbone1Seg1Ptr_->getActor()->GetProperty()->SetLineWidth(4.0);
    backbone2Seg1Ptr_->getActor()->GetProperty()->SetColor(0,1,0);
    backbone2Seg1Ptr_->getActor()->GetProperty()->SetLineWidth(4.0);
    backbone3Seg1Ptr_->getActor()->GetProperty()->SetColor(0,0,1);
    backbone3Seg1Ptr_->getActor()->GetProperty()->SetLineWidth(4.0);
    backbone1Seg2Ptr_->getActor()->GetProperty()->SetColor(1,0,0);
    backbone1Seg2Ptr_->getActor()->GetProperty()->SetLineWidth(4.0);
    backbone2Seg2Ptr_->getActor()->GetProperty()->SetColor(0,1,0);
    backbone2Seg2Ptr_->getActor()->GetProperty()->SetLineWidth(4.0);
    backbone3Seg2Ptr_->getActor()->GetProperty()->SetColor(0,0,1);
    backbone3Seg2Ptr_->getActor()->GetProperty()->SetLineWidth(4.0);
    backbone1Seg3Ptr_->getActor()->GetProperty()->SetColor(1,0,0);
    backbone1Seg3Ptr_->getActor()->GetProperty()->SetLineWidth(4.0);
    backbone2Seg3Ptr_->getActor()->GetProperty()->SetColor(0,1,0);
    backbone2Seg3Ptr_->getActor()->GetProperty()->SetLineWidth(4.0);
    backbone3Seg3Ptr_->getActor()->GetProperty()->SetColor(0,0,1);
    backbone3Seg3Ptr_->getActor()->GetProperty()->SetLineWidth(4.0);
}

std::vector<double> VtkMBCRobot::getRobotXform(const double psi[7])
{

    double lengthArr[] = {length_[0],length_[1],length_[2]};
    get_Robot_Xform(psi,lengthArr,xFormVec_);

    // put xFormVec_ in std::vector
    std::vector<double> xFormVec__(91);
    for (int i = 0;i<91;i++)
    {
        xFormVec__.at(i) = xFormVec_[i];
    }
    return xFormVec__;

}

void VtkMBCRobot::setRobotXform(double psi[7])
{

    // get poses of disks
    std::vector<double> xForm_Vec = getRobotXform(psi);

    // update poses of disks
    double Frame_Quat[7];
    for (int i=1;i<13;i++)
    {
        for (int j=0;j<7;j++) Frame_Quat[j] = xForm_Vec.at(7*i+j);

        if (i==1)
        {
            disk1_seg1_->setPose(Frame_Quat);
        }

        if (i==2)
        {
            disk2_seg1_->setPose(Frame_Quat);
        }

        if (i==3)
        {
            disk3_seg1_->setPose(Frame_Quat);
        }

        if (i==4)
        {
            disk4_seg1_->setPose(Frame_Quat);
        }

        if (i==5)
        {
            disk5_seg1_->setPose(Frame_Quat);
        }

        if (i==6)
        {
            disk1_seg2_->setPose(Frame_Quat);
        }
        if (i==7)
        {
            disk2_seg2_->setPose(Frame_Quat);
        }

        if (i==8)
        {
            disk3_seg2_->setPose(Frame_Quat);
        }

        if (i==9)
        {
            disk4_seg2_->setPose(Frame_Quat);
        }

        if (i==10)
        {
            disk1_seg3_->setPose(Frame_Quat);
        }

        if (i==11)
        {
            disk2_seg3_->setPose(Frame_Quat);
        }

        if (i==12)
        {
            disk3_seg3_->setPose(Frame_Quat);
        }

    }

    // update segment1 backbones shapes
    continuum_robot::ConfigSingleSeg seg1Config;
    seg1Config.theta = psi[0];
    seg1Config.delta = psi[1];
    backbone1Seg1Ptr_->genShapeMapper(seg1Config);
    backbone2Seg1Ptr_->genShapeMapper(seg1Config);
    backbone3Seg1Ptr_->genShapeMapper(seg1Config);

    // update segment2 backbones shapes
    continuum_robot::ConfigSingleSeg seg2Config;
    seg2Config.theta = psi[2];
    seg2Config.delta = psi[3];
    backbone1Seg2Ptr_->genShapeMapper(seg2Config);
    backbone2Seg2Ptr_->genShapeMapper(seg2Config);
    backbone3Seg2Ptr_->genShapeMapper(seg2Config);

    // update segment3 backbones shapes
    continuum_robot::ConfigSingleSeg seg3Config;
    seg3Config.theta = psi[4];
    seg3Config.delta = psi[5];
    backbone1Seg3Ptr_->genShapeMapper(seg3Config);
    backbone2Seg3Ptr_->genShapeMapper(seg3Config);
    backbone3Seg3Ptr_->genShapeMapper(seg3Config);

    //
    double lengthArr[] = {length_[0],length_[1],length_[2]};

    // set frame of segment 1 backbones
    double frameI = 0;
    double frameJ = 1;
    double homXform01[16];
    directKin(psi, frameI, frameJ, lengthArr, homXform01);
    Eigen::Matrix4d xform;
    xform << homXform01[0],homXform01[4],homXform01[8],homXform01[12],
            homXform01[1],homXform01[5],homXform01[9],homXform01[13],
            homXform01[2],homXform01[6],homXform01[10],homXform01[14],
            homXform01[3],homXform01[7],homXform01[11],homXform01[15];
    backbone1Seg1Ptr_->setTransform(xform);
    backbone2Seg1Ptr_->setTransform(xform);
    backbone3Seg1Ptr_->setTransform(xform);

    // set frame of segment 2 backbones
    frameJ = 2;
    double homXform02[16];
    directKin(psi, frameI, frameJ, lengthArr, homXform02);
    //Eigen::Matrix4d xform;
    xform << homXform02[0],homXform02[4],homXform02[8],homXform02[12],
            homXform02[1],homXform02[5],homXform02[9],homXform02[13],
            homXform02[2],homXform02[6],homXform02[10],homXform02[14],
            homXform02[3],homXform02[7],homXform02[11],homXform02[15];
    backbone1Seg2Ptr_->setTransform(xform);
    backbone2Seg2Ptr_->setTransform(xform);
    backbone3Seg2Ptr_->setTransform(xform);

    // set frame of segment 3 backbones
    frameJ = 3;
    double homXform03[16];
    directKin(psi, frameI, frameJ, lengthArr, homXform03);
    //Eigen::Matrix4d xform;
    xform << homXform03[0],homXform03[4],homXform03[8],homXform03[12],
            homXform03[1],homXform03[5],homXform03[9],homXform03[13],
            homXform03[2],homXform03[6],homXform03[10],homXform03[14],
            homXform03[3],homXform03[7],homXform03[11],homXform03[15];
    backbone1Seg3Ptr_->setTransform(xform);
    backbone2Seg3Ptr_->setTransform(xform);
    backbone3Seg3Ptr_->setTransform(xform);
}

vtkSmartPointer<vtkRenderer> VtkMBCRobot::getRenderer()
{
    return renderer_;
}


