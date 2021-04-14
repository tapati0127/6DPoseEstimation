#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "camera.h"
#include "pclviewer.h"
#include "handeyecalibration.h"
#include "servocontrol.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
public slots:
    // Slot that will receive frames from the camera
    void receiveFrame(QImage rgb, QImage depth);
    void receivePcl(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud);
    void receivePointCloud(cv::Mat pointcloud);
    void addCoordinate(Eigen::Affine3f aswer);
private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

private:
    Ui::MainWindow *ui;
    Camera *camera;
    pclViewer *viewer;
    HandEyeCalibration *calib;
    ServoControl* servo;
};

#endif // MAINWINDOW_H
