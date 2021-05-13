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
#include "ppf.h"
#include "yolo.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    void init();
    ~MainWindow();
    void choosePointCloud(const vector<struct Yolo::yoloResult>&,const cv::Mat&,const rs2_intrinsics&,cv::Mat &);
public slots:
    // Slot that will receive frames from the camera
    void receiveFrame(QImage rgb);
    void receivePcl(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud);
    void receivePointCloud(cv::Mat pointcloud,cv::Mat depth_image);
    void addCoordinate(Eigen::Affine3f aswer);
    void ReadSettings();
    void WriteSettings();
    void readyCameraStatus();
    void readyPPFStatus();
    void triggerByRobot();
private slots:
    void on_pushButtonSaveSettings_clicked();

private slots:
    void on_toolButtonYoloPath_clicked();

private slots:
    void on_pushButtonServoOn_clicked();

private slots:
    void on_pushButtonStart_clicked();

private slots:
    void on_pushButtonSaveImage_clicked();

private slots:
    void on_radioButtonTrigger_toggled(bool checked);

private slots:
    void on_verticalSlider_valueChanged(int value);

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
    PPF* ppf;
    Yolo* yolo;
    bool isRuntime = true;

    QString robotIP;
    QString jobName;
    QString gripperAddress;
    int firstPV, firstBV;
    QString calibFile;
    QString camFile;
    QString modelPath;
    QString yoloPath;
    bool isTrigger = false;
    bool isReady = false;
    bool isCameraRunning = false;
    cv::Matx44d calib_pose,calib_pose_;

};

#endif // MAINWINDOW_H
