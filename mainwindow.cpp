#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //vtkImageData::GlobalWarningDisplayOff(); // suppress VTK warnings
    ui->setupUi(this);

    double theta1L_in_degree = ui->theta1lLineEdit->text().toDouble();
    Psi_Global[0] = (::PI/180.0)*theta1L_in_degree;
    // std::cout << "theta1L " << Psi_Global[0] << endl;
    double delta1L_in_degree = ui->delta1LineEdit->text().toDouble();
    Psi_Global[1] = (::PI/180.0)*delta1L_in_degree;
    double theta2L_in_degree = ui->theta2lLineEdit->text().toDouble();
    Psi_Global[2] = (::PI/180.0)*theta2L_in_degree;
    double delta2L_in_degree = ui->delta2LineEdit->text().toDouble();
    Psi_Global[3] = (::PI/180.0)*delta2L_in_degree;
    double theta3L_in_degree = ui->theta3lLineEdit->text().toDouble();
    Psi_Global[4] = (::PI/180.0)*theta3L_in_degree;
    double delta3L_in_degree = ui->delta3LineEdit->text().toDouble();
    Psi_Global[5] = (::PI/180.0)*delta3L_in_degree;
    double qIns_in_mm = ui->qInsLineEdit->text().toDouble();
    Psi_Global[6] = qIns_in_mm;

    //ui->displayLabel->setText("Teleoperation Input");
    int port = 32000;
    socket1_ = configureUDP(port);

    //
    ui->qvtkWidget->GetRenderWindow()->AddRenderer(robot_.getRenderer());
    ui->qvtkWidget->update();

    // signals and slots
    // to update config from GUI input
    connect(ui->updatePushButton,SIGNAL(clicked()),this,SLOT(updateConfigFromGUI()));

    // to update vtk render from GUI input
    connect(this,SIGNAL(robotConfigUpdatedFromGui()),this,SLOT(updateVtkRender()));

    // to update config from udp
    connect(socket1_,SIGNAL(readyRead()),this,SLOT(updateConfigFromUDP()));

    // to update vtk render from udp when the timer times out
    QTimer *timerRenderUpdate = new QTimer();// timer for updating vtk widget
    timerRenderUpdate->start(10);
    connect(timerRenderUpdate,SIGNAL(timeout()),this,SLOT(updateVtkRender()));

}

MainWindow::~MainWindow()
{
    delete ui;
}

QUdpSocket* MainWindow::configureUDP(int port)
{
    QUdpSocket* socket = new QUdpSocket(this);
    socket->bind(port);
    return socket;
}

//void MainWindow::sendUdpTest()
//{
//    double psi[7] = {1,0,2,0,2.5,0,0};
//    QByteArray datagram;
//    QDataStream out(&datagram, QIODevice::WriteOnly);
//    out.setVersion(QDataStream::Qt_5_5);
//    out << psi[0] << psi[1]<< psi[2] << psi[3]<< psi[4] << psi[5]<< psi[6];
//    socket1_->writeDatagram(datagram,QHostAddress::LocalHost, 32000);
//}

void MainWindow::updateConfigFromGUI()
{
    double theta1L_in_degree = ui->theta1lLineEdit->text().toDouble();
    ::Psi_Global[0] = (::PI/180.0)*theta1L_in_degree;
    double delta1L_in_degree = ui->delta1LineEdit->text().toDouble();
    ::Psi_Global[1] = (::PI/180.0)*delta1L_in_degree;
    double theta2L_in_degree = ui->theta2lLineEdit->text().toDouble();
    ::Psi_Global[2] = (::PI/180.0)*theta2L_in_degree;
    double delta2L_in_degree = ui->delta2LineEdit->text().toDouble();
    ::Psi_Global[3] = (::PI/180.0)*delta2L_in_degree;
    double theta3L_in_degree = ui->theta3lLineEdit->text().toDouble();
    ::Psi_Global[4] = (::PI/180.0)*theta3L_in_degree;
    double delta3L_in_degree = ui->delta3LineEdit->text().toDouble();
    ::Psi_Global[5] = (::PI/180.0)*delta3L_in_degree;
    double qIns_in_mm = ui->qInsLineEdit->text().toDouble();
    ::Psi_Global[6] = qIns_in_mm;

    emit robotConfigUpdatedFromGui();
}

void MainWindow::updateConfigFromUDP()
{
    // Read udp datagram and store in Psi_Global
    QByteArray buffer;
    buffer.resize(socket1_->pendingDatagramSize());
    QHostAddress sender;
    quint16 sender_port;
    socket1_->readDatagram(buffer.data(), buffer.size(), &sender, &sender_port);

    QDataStream in(&buffer, QIODevice::ReadOnly);
    in.setVersion(QDataStream::Qt_5_5);
    in.setByteOrder(QDataStream::LittleEndian);
    //in >> ::Psi_Global[0]>> ::Psi_Global[1]>> ::Psi_Global[2]>> ::Psi_Global[3]>> ::Psi_Global[4]>> ::Psi_Global[5]>> ::Psi_Global[6]>> ::Psi_Global[7];
    in >> ::Psi_Global[0]
            >> ::Psi_Global[1]
            >> ::Psi_Global[2]
            >> ::Psi_Global[3]
            >> ::Psi_Global[4]
            >> ::Psi_Global[5]
            >> ::Psi_Global[6];
    ::Psi_Global[6] = ::Psi_Global[6]*1000.0; // convert to mm

    // update display text on GUI
    ui->udp_TextEdit->setText("Recieved theta1L="+QString::number((180.0/::PI)*::Psi_Global[0])+", delta1="+QString::number((180.0/::PI)*::Psi_Global[1])+
            ", theta2L="+QString::number((180.0/::PI)*::Psi_Global[2])+", delta2="+QString::number((180.0/::PI)*::Psi_Global[3])+
            ", theta3L="+QString::number((180.0/::PI)*::Psi_Global[4])+", delta3="+QString::number((180.0/::PI)*::Psi_Global[5])+
            ", qIns(mm)="+QString::number(::Psi_Global[6]));
}

void MainWindow::updateVtkRender()
{
    // update rigid transforms in MBCR
    robot_.setRobotXform(::Psi_Global);

    // update vtk
    ui->qvtkWidget->update();
}


