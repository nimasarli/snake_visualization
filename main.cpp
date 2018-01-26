// ALL UNITS IN MM and RADIANS
#include "mainwindow.h"
#include <QApplication>
#include <vtkAutoInit.h>
// if not using CMake to compile, necessary to use these macros
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkRenderingFreeType)

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
