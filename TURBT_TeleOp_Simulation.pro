#-------------------------------------------------
#
# Project created by QtCreator 2014-10-31T14:55:05
#
#-------------------------------------------------

QT       += core gui network opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = TURBT_TeleOp_Simulation
TEMPLATE = app
#CONFIG += embed_manifest_exe

### External header files paths
## If it is added to environment variable use $$()
message(VTK directory is $$(VTK_DIR))
INCLUDEPATH += $$(VTK_DIR)\include\vtk-6.1

message(Eigen directory is $$(EIGEN_DIR))
INCLUDEPATH += $$(EIGEN_DIR)

### External lib files paths
QMAKE_LIBDIR += $$(VTK_LIBS)
#Release: QMAKE_LIBDIR += $$(VTK_LIBS)
#Debug: QMAKE_LIBDIR +=  $$(VTK_LIBS)

### Headers
HEADERS  += mainwindow.h \
            vtksolid.h \
            vtkmbcrobot.h \
            vtksolid.h \
            rtwtypes.h \
            get_robot_xform.h \
            get_robot_xform_types.h \
            globals.h \
            robotic.h \
            vtkbackbone.h \
    continuum_robot.h


### Source Files
SOURCES += main.cpp\
           mainwindow.cpp \
           vtksolid.cpp \
           vtkmbcrobot.cpp \
           get_robot_xform.cpp \
           globals.cpp \
           robotic.cpp \
           vtkbackbone.cpp \
    continuum_robot.cpp

### Forms
FORMS    += mainwindow.ui

### External Libraries
LIBS += 	vtkChartsCore-6.1.lib \
                vtkCommonColor-6.1.lib \
                vtkCommonComputationalGeometry-6.1.lib \
                vtkCommonCore-6.1.lib \
                vtkCommonDataModel-6.1.lib \
                vtkCommonExecutionModel-6.1.lib \
                vtkCommonMath-6.1.lib \
                vtkCommonMisc-6.1.lib \
                vtkCommonSystem-6.1.lib \
                vtkCommonTransforms-6.1.lib \
                vtkDICOMParser-6.1.lib \
                vtkDomainsChemistry-6.1.lib \
                vtkFiltersAMR-6.1.lib \
                vtkFiltersCore-6.1.lib \
                vtkFiltersExtraction-6.1.lib \
                vtkFiltersFlowPaths-6.1.lib \
                vtkFiltersGeneral-6.1.lib \
                vtkFiltersGeneric-6.1.lib \
                vtkFiltersGeometry-6.1.lib \
                vtkFiltersHybrid-6.1.lib \
                vtkFiltersHyperTree-6.1.lib \
                vtkFiltersImaging-6.1.lib \
                vtkFiltersModeling-6.1.lib \
                vtkFiltersParallel-6.1.lib \
                vtkFiltersParallelImaging-6.1.lib \
                vtkFiltersProgrammable-6.1.lib \
                vtkFiltersSMP-6.1.lib \
                vtkFiltersSelection-6.1.lib \
                vtkFiltersSources-6.1.lib \
                vtkFiltersStatistics-6.1.lib \
                vtkFiltersTexture-6.1.lib \
                vtkFiltersVerdict-6.1.lib \
                vtkGUISupportQt-6.1.lib \
                vtkGUISupportQtOpenGL-6.1.lib \
                vtkGUISupportQtSQL-6.1.lib \
                vtkGUISupportQtWebkit-6.1.lib \
                vtkGeovisCore-6.1.lib \
                vtkIOAMR-6.1.lib \
                vtkIOCore-6.1.lib \
                vtkIOEnSight-6.1.lib \
                vtkIOExodus-6.1.lib \
                vtkIOExport-6.1.lib \
                vtkIOGeometry-6.1.lib \
                vtkIOImage-6.1.lib \
                vtkIOImport-6.1.lib \
                vtkIOInfovis-6.1.lib \
                vtkIOLSDyna-6.1.lib \
                vtkIOLegacy-6.1.lib \
                vtkIOMINC-6.1.lib \
                vtkIOMovie-6.1.lib \
                vtkIONetCDF-6.1.lib \
                vtkIOPLY-6.1.lib \
                vtkIOParallel-6.1.lib \
                vtkIOSQL-6.1.lib \
                vtkIOVideo-6.1.lib \
                vtkIOXML-6.1.lib \
                vtkIOXMLParser-6.1.lib \
                vtkImagingColor-6.1.lib \
                vtkImagingCore-6.1.lib \
                vtkImagingFourier-6.1.lib \
                vtkImagingGeneral-6.1.lib \
                vtkImagingHybrid-6.1.lib \
                vtkImagingMath-6.1.lib \
                vtkImagingMorphological-6.1.lib \
                vtkImagingSources-6.1.lib \
                vtkImagingStatistics-6.1.lib \
                vtkImagingStencil-6.1.lib \
                vtkInfovisCore-6.1.lib \
                vtkInfovisLayout-6.1.lib \
                vtkInteractionImage-6.1.lib \
                vtkInteractionStyle-6.1.lib \
                vtkInteractionWidgets-6.1.lib \
                vtkNetCDF-6.1.lib \
                vtkNetCDF_cxx-6.1.lib \
                vtkParallelCore-6.1.lib \
                vtkRenderingAnnotation-6.1.lib \
                vtkRenderingContext2D-6.1.lib \
                vtkRenderingCore-6.1.lib \
                vtkRenderingFreeType-6.1.lib \
                vtkRenderingFreeTypeOpenGL-6.1.lib \
                vtkRenderingGL2PS-6.1.lib \
                vtkRenderingImage-6.1.lib \
                vtkRenderingLIC-6.1.lib \
                vtkRenderingLOD-6.1.lib \
                vtkRenderingLabel-6.1.lib \
                vtkRenderingOpenGL-6.1.lib \
                vtkRenderingQt-6.1.lib \
                vtkRenderingVolume-6.1.lib \
                vtkRenderingVolumeAMR-6.1.lib \
                vtkRenderingVolumeOpenGL-6.1.lib \
                vtkViewsContext2D-6.1.lib \
                vtkViewsCore-6.1.lib \
                vtkViewsGeovis-6.1.lib \
                vtkViewsInfovis-6.1.lib \
                vtkViewsQt-6.1.lib \
                vtkalglib-6.1.lib \
                vtkexoIIc-6.1.lib \
                vtkexpat-6.1.lib \
                vtkfreetype-6.1.lib \
                vtkftgl-6.1.lib \
                vtkgl2ps-6.1.lib \
                vtkhdf5-6.1.lib \
                vtkhdf5_hl-6.1.lib \
                vtkjpeg-6.1.lib \
                vtkjsoncpp-6.1.lib \
                vtklibxml2-6.1.lib \
                vtkmetaio-6.1.lib \
                vtkoggtheora-6.1.lib \
                vtkpng-6.1.lib \
                vtkproj4-6.1.lib \
                vtksqlite-6.1.lib \
                vtksys-6.1.lib \
                vtktiff-6.1.lib \
                vtkverdict-6.1.lib \
                vtkzlib-6.1.lib \


