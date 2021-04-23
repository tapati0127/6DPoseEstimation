#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QFileDialog>
#include "camera.h"
#include "pclviewer.h"
#include "handeyecalibration.h"
#include "servocontrol.h"
#include "motoudp.h"
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
    void ReadSettings();
    void WriteSettings();
private slots:
    void on_pushButtonStartSave_clicked();

private slots:
    void on_pushButtonStartCalib_clicked();

private slots:
    void on_toolButtonModelPath_clicked();

private slots:
    void on_toolButtonCalibCameraParam_clicked();

private slots:
    void on_toolButtonCalib_clicked();

private slots:
    void on_pushButtonCaculate_clicked();

private slots:
    void on_pushButtonGetPose_clicked();

private slots:
    void on_pushButtonSendPosition_clicked();

private slots:
    void on_pushButton_3_clicked();

private:
    Ui::MainWindow *ui;
    Camera *camera;
    pclViewer *viewer;
    pclViewer *viewer_;
    HandEyeCalibration *calib;
    ServoControl* servo;
    MotoUDP* motoudp;
    bool isRuntime = true;

    QString robotIP;
    QString jobName;
    QString gripperAddress;
    int firstPV, firstBV;
    QString calibFile;
    QString camFile;
    QString modelPath;

};

#endif // MAINWINDOW_H
