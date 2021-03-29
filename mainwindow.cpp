#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "camera.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    /*
    camera = new Camera(848, 480, 848, 480, 30);
    connect(camera, &Camera::framesReady, this, &MainWindow::receiveFrame);
    connect(camera, &Camera::pclReady, this, &MainWindow::receivePcl);
    connect(camera, &Camera::pointCloudReady, this, &MainWindow::receivePointCloud);
    camera->start();*/
    viewer = new pclViewer(ui->qvtkWidget);
    //viewer.display(ui->qvtkWidget);
    calib = new HandEyeCalibration;
    connect(calib, &HandEyeCalibration::framesReady, this, &MainWindow::receiveFrame);
    calib->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::receiveFrame(QImage rgb, QImage depth)
{
    ui->labelImage->setPixmap(QPixmap::fromImage(rgb).scaled(ui->labelImage->size().width(),ui->labelImage->size().height(),Qt::KeepAspectRatio));
    QString sizeString = QString("(%1,%2)").arg(ui->labelImage->size().width()).arg(ui->labelImage->size().height());
    //ui->labelSize->setText(sizeString);
}

void MainWindow::receivePcl(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud)
{
    //std::cout <<"Got PCL: " << pointcloud->points.size() << std::endl;
    viewer->displayPCL(pointcloud);
}

void MainWindow::receivePointCloud(cv::Mat pointcloud)
{

}
