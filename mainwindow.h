#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVTKWidget.h>
#include <vtkImageData.h>
#include "vtkmbcrobot.h"
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkProperty.h>
//#include <QDoubleValidator>
#include <QUdpSocket>
#include <Qtimer>
#include "globals.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QUdpSocket* configureUDP(int port);
    //void sendUdpTest();

private slots:
    void updateConfigFromGUI();
    void updateConfigFromUDP();
    void updateVtkRender();
    //void updateTeleOp(double quat_m[8]);
    //void readyRead();

signals:
    void robotConfigUpdatedFromGui();

private:
    Ui::MainWindow *ui;
    VtkMBCRobot robot_;
    QUdpSocket *socket1_;
};

#endif // MAINWINDOW_H
